/*
 * touch-wiggle - keep the lock screen away, with a touch toggle.
 *
 * The Tomu enumerates as a USB mouse. While "armed" (green LED) it nudges the
 * pointer one pixel and back once a minute so the screen never locks. Tapping
 * the capacitive touch pad toggles the state: green (armed, wiggling) <-> red
 * (paused, idle).
 *
 * Structure:
 *   - USB HID mouse descriptors are adapted from the usb-hid example.
 *   - The capacitive-sense plumbing (ACMP + TIMER0/TIMER1 + PRS) is the proven
 *     setup shared by the captouch / coinflip examples.
 *   - main() polls the touch pad for edge-triggered toggles; a SysTick timer
 *     counts out the once-a-minute wiggle.
 */

#include <libopencm3/cm3/common.h>
#include <libopencm3/cm3/vector.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>
#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/efm32/wdog.h>
#include <libopencm3/efm32/gpio.h>
#include <libopencm3/efm32/cmu.h>
#include <libopencm3/efm32/timer.h>
#include <libopencm3/efm32/common/prs_common.h>
#include <libopencm3/efm32/common/acmp_common.h>

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "captouch.h"
#include "capsenseconfig.h"

/* Make this program compatible with Toboot-V2.0.
 * Use TOBOOT_CONFIGURATION(0) so the bootloader is easy to re-enter while
 * iterating (just re-plug). Switch to TOBOOT_CONFIG_FLAG_AUTORUN once you're
 * happy and want it to start automatically on insert. */
#include <toboot.h>
TOBOOT_CONFIGURATION(0);

/* Systick interrupt frequency, Hz */
#define SYSTICK_FREQUENCY 100

/* Default AHB (core clock) frequency of Tomu board */
#define AHB_FREQUENCY 14000000

/* Wiggle the pointer once every WIGGLE_PERIOD_TICKS systicks (100 Hz -> 60s) */
#define WIGGLE_PERIOD_TICKS (60 * SYSTICK_FREQUENCY)

/* How far to nudge the pointer (pixels). Moved out and straight back. */
#define WIGGLE_PIXELS 1

#define LED_GREEN_PORT GPIOA
#define LED_GREEN_PIN  GPIO0
/* Dim the green LED by limiting its GPIO drive current rather than PWMing it.
 * The pin level stays static, so (unlike PWM) it injects no switching noise
 * into the capacitive sense. Only PA0 lives on port A, so lowering port A's
 * drive strength affects nothing else; red (port B) stays full-bright.
 * Tune brightness here: GPIO_STRENGTH_LOWEST (~0.5 mA) is dimmest,
 * GPIO_STRENGTH_LOW (~1 mA) a step brighter. */
#define GREEN_DRIVE_STRENGTH GPIO_STRENGTH_LOWEST
#define LED_RED_PORT   GPIOB
#define LED_RED_PIN    GPIO7
#define CAP0B_PORT     GPIOE
#define CAP0B_PIN      GPIO12
#define CAP1B_PORT     GPIOE
#define CAP1B_PIN      GPIO13

#define VENDOR_ID                 0x1209    /* pid.code */
#define PRODUCT_ID                0x70b1    /* Assigned to Tomu project */
#define DEVICE_VER                0x0101    /* Program version */

/* Minimum summed capsense value for a touch to register (same as coinflip). */
#define CAPSENSE_DETECT_MIN 20

/* Touched generations required to toggle, so brief/accidental contact is
 * ignored. The capsense reading flickers across the threshold even during a
 * solid press, so we count touched generations *up* and don't subtract on
 * isolated misses (see main loop) -- this makes the value map directly to
 * touch time (~600 gens/s, so ~40 gens is well under 1/4 s). */
#define TOUCH_HOLD_GENS 40

/* Consecutive below-threshold generations that count as a real release (and
 * reset the touch counter). Small, since lift-off reads clean. */
#define TOUCH_RELEASE_GENS 8

#pragma warning "Re-defining TIMER_CC_CTRL_INSEL because it's wrong"
#undef TIMER_CC_CTRL_INSEL
#define TIMER_CC_CTRL_INSEL (1 << 20)

#define EFM_ASSERT(x)

/* ---- capsense state (shared with the TIMER0 ISR) ----------------------- */

/* Latest read values from the ACMP, per channel. */
static volatile uint32_t g_channel_values[4] = {0};

/* Maximum values ever seen, per channel. */
static volatile uint32_t channelMaxValues[4] = {0};

/* The channel currently being sensed. */
static volatile uint8_t g_current_channel;

/* Monotonically increasing counter, bumped once per full sweep of channels. */
static volatile uint32_t g_capsense_generation;

/* True while capsense is free-running. */
static volatile bool g_capsense_running = false;

/* ---- application state ------------------------------------------------- */

/* true = green (armed, wiggling), false = red (paused). Starts armed. */
static volatile bool g_armed = true;

/* Set by SysTick when it's time to wiggle; consumed by the main loop. */
static volatile bool g_do_wiggle = false;

/* ---- forward declarations ---------------------------------------------- */

static void ACMP_CapsenseChannelSet(uint32_t channel);
static void CAPSENSE_Measure(uint32_t channel);
void timer0_isr(void);
void capsense_start(void);
void capsense_stop(void);
void setup_acmp_capsense(const struct acmp_capsense_init *init);
static void setup_capsense(void);
static void setup(void);

bool g_usbd_is_connected = false;
usbd_device *g_usbd_dev = 0;

/* ---- USB HID mouse descriptors (from the usb-hid example) -------------- */

static const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = VENDOR_ID,
	.idProduct = PRODUCT_ID,
	.bcdDevice = DEVICE_VER,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

static const uint8_t hid_report_descriptor[] = {
	0x05, 0x01, /* USAGE_PAGE (Generic Desktop)         */
	0x09, 0x02, /* USAGE (Mouse)                        */
	0xa1, 0x01, /* COLLECTION (Application)             */
	0x09, 0x01, /*   USAGE (Pointer)                    */
	0xa1, 0x00, /*   COLLECTION (Physical)              */
	0x05, 0x09, /*     USAGE_PAGE (Button)              */
	0x19, 0x01, /*     USAGE_MINIMUM (Button 1)         */
	0x29, 0x03, /*     USAGE_MAXIMUM (Button 3)         */
	0x15, 0x00, /*     LOGICAL_MINIMUM (0)              */
	0x25, 0x01, /*     LOGICAL_MAXIMUM (1)              */
	0x95, 0x03, /*     REPORT_COUNT (3)                 */
	0x75, 0x01, /*     REPORT_SIZE (1)                  */
	0x81, 0x02, /*     INPUT (Data,Var,Abs)             */
	0x95, 0x01, /*     REPORT_COUNT (1)                 */
	0x75, 0x05, /*     REPORT_SIZE (5)                  */
	0x81, 0x01, /*     INPUT (Cnst,Ary,Abs)             */
	0x05, 0x01, /*     USAGE_PAGE (Generic Desktop)     */
	0x09, 0x30, /*     USAGE (X)                        */
	0x09, 0x31, /*     USAGE (Y)                        */
	0x09, 0x38, /*     USAGE (Wheel)                    */
	0x15, 0x81, /*     LOGICAL_MINIMUM (-127)           */
	0x25, 0x7f, /*     LOGICAL_MAXIMUM (127)            */
	0x75, 0x08, /*     REPORT_SIZE (8)                  */
	0x95, 0x03, /*     REPORT_COUNT (3)                 */
	0x81, 0x06, /*     INPUT (Data,Var,Rel)             */
	0xc0,       /*   END_COLLECTION                     */
	0x09, 0x3c, /*   USAGE (Motion Wakeup)              */
	0x05, 0xff, /*   USAGE_PAGE (Vendor Defined Page 1) */
	0x09, 0x01, /*   USAGE (Vendor Usage 1)             */
	0x15, 0x00, /*   LOGICAL_MINIMUM (0)                */
	0x25, 0x01, /*   LOGICAL_MAXIMUM (1)                */
	0x75, 0x01, /*   REPORT_SIZE (1)                    */
	0x95, 0x02, /*   REPORT_COUNT (2)                   */
	0xb1, 0x22, /*   FEATURE (Data,Var,Abs,NPrf)        */
	0x75, 0x06, /*   REPORT_SIZE (6)                    */
	0x95, 0x01, /*   REPORT_COUNT (1)                   */
	0xb1, 0x01, /*   FEATURE (Cnst,Ary,Abs)             */
	0xc0        /* END_COLLECTION                       */
};

static const struct {
	struct usb_hid_descriptor hid_descriptor;
	struct {
		uint8_t bReportDescriptorType;
		uint16_t wDescriptorLength;
	} __attribute__((packed)) hid_report;
} __attribute__((packed)) hid_function = {
	.hid_descriptor = {
		.bLength = sizeof(hid_function),
		.bDescriptorType = USB_DT_HID,
		.bcdHID = 0x0100,
		.bCountryCode = 0,
		.bNumDescriptors = 1,
	},
	.hid_report = {
		.bReportDescriptorType = USB_DT_REPORT,
		.wDescriptorLength = sizeof(hid_report_descriptor),
	}
};

const struct usb_endpoint_descriptor hid_endpoint = {
	.bLength = USB_DT_ENDPOINT_SIZE,
	.bDescriptorType = USB_DT_ENDPOINT,
	.bEndpointAddress = 0x81,
	.bmAttributes = USB_ENDPOINT_ATTR_INTERRUPT,
	.wMaxPacketSize = 4,
	.bInterval = 0x20,
};

const struct usb_interface_descriptor hid_iface = {
	.bLength = USB_DT_INTERFACE_SIZE,
	.bDescriptorType = USB_DT_INTERFACE,
	.bInterfaceNumber = 0,
	.bAlternateSetting = 0,
	.bNumEndpoints = 1,
	.bInterfaceClass = USB_CLASS_HID,
	.bInterfaceSubClass = 1, /* boot */
	.bInterfaceProtocol = 2, /* mouse */
	.iInterface = 0,

	.endpoint = &hid_endpoint,

	.extra = &hid_function,
	.extralen = sizeof(hid_function),
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = 1,
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xC0,
	.bMaxPower = 0x32,
	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Tomu",
	"Touch Wiggle",
	"WIGGLE",
};

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

static enum usbd_request_return_codes hid_control_request(usbd_device *dev, struct usb_setup_data *req, uint8_t **buf, uint16_t *len,
			void (**complete)(usbd_device *, struct usb_setup_data *))
{
	(void)complete;
	(void)dev;

	if((req->bmRequestType != 0x81) ||
	   (req->bRequest != USB_REQ_GET_DESCRIPTOR) ||
	   (req->wValue != 0x2200))
		return 0;

	/* Handle the HID report descriptor. */
	*buf = (uint8_t *)hid_report_descriptor;
	*len = sizeof(hid_report_descriptor);

	/* Dirty way to know if we're connected */
	g_usbd_is_connected = true;

	return 1;
}

static void hid_set_config(usbd_device *dev, uint16_t wValue)
{
	(void)wValue;
	(void)dev;

	usbd_ep_setup(dev, 0x81, USB_ENDPOINT_ATTR_INTERRUPT, 4, NULL);

	usbd_register_control_callback(
				dev,
				USB_REQ_TYPE_STANDARD | USB_REQ_TYPE_INTERFACE,
				USB_REQ_TYPE_TYPE | USB_REQ_TYPE_RECIPIENT,
				hid_control_request);
}

void usb_isr(void)
{
	usbd_poll(g_usbd_dev);
}

void hard_fault_handler(void)
{
	while(1);
}

/* ---- LED helpers ------------------------------------------------------- *
 * The LEDs are wired active-low (GPIO_MODE_WIRED_AND), so clearing the pin
 * turns the LED on and setting it turns it off. */

static void led_green(bool on)
{
	if (on)
		gpio_clear(LED_GREEN_PORT, LED_GREEN_PIN);
	else
		gpio_set(LED_GREEN_PORT, LED_GREEN_PIN);
}

static void led_red(bool on)
{
	if (on)
		gpio_clear(LED_RED_PORT, LED_RED_PIN);
	else
		gpio_set(LED_RED_PORT, LED_RED_PIN);
}

/* Reflect g_armed on the LEDs: green when armed, red when paused. */
static void update_leds(void)
{
	led_green(g_armed);
	led_red(!g_armed);
}

/* Send a single relative mouse movement report. */
static void mouse_move(int8_t dx, int8_t dy)
{
	uint8_t buf[4] = {0, (uint8_t)dx, (uint8_t)dy, 0};
	usbd_ep_write_packet(g_usbd_dev, 0x81, buf, 4);
}

/* SysTick: while armed and connected, count out the wiggle interval. */
void sys_tick_handler(void)
{
	static uint32_t ticks = 0;

	if (g_usbd_is_connected && g_armed) {
		if (++ticks >= WIGGLE_PERIOD_TICKS) {
			ticks = 0;
			g_do_wiggle = true;
		}
	} else {
		/* Restart the minute whenever we're paused or unplugged. */
		ticks = 0;
	}
}

/* Nudge the pointer out and straight back, so it stays put. */
static void do_wiggle(void)
{
	mouse_move(WIGGLE_PIXELS, 0);
	/* Brief pause so the host registers two distinct movements. */
	for (int i = 0; i != 100000; ++i)
		__asm__("nop");
	mouse_move(-WIGGLE_PIXELS, 0);
}

int main(void)
{
	uint32_t last_generation = 0;
	uint32_t touch_count = 0;        /* touched generations accumulated this press */
	uint32_t idle_run = 0;           /* consecutive below-threshold generations */
	bool toggled_this_hold = false;  /* already toggled for the current press */

	/* Make sure the vector table is relocated correctly (after the Tomu bootloader) */
	SCB_VTOR = 0x4000;

	/* Disable the watchdog that the bootloader started. */
	WDOG_CTRL = 0;

	/* GPIO, LEDs and capsense peripherals. */
	setup();

	/* Show the initial (armed) state. */
	update_leds();

	/* Start the capacitive sensing free-runner. */
	capsense_start();

	/* Configure the USB core & stack. */
	g_usbd_dev = usbd_init(&efm32hg_usb_driver, &dev_descr, &config, usb_strings, 3, usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(g_usbd_dev, hid_set_config);

	/* Enable USB IRQs. */
	nvic_set_priority(NVIC_USB_IRQ, 0x40);
	nvic_enable_irq(NVIC_USB_IRQ);

	/* SysTick drives the once-a-minute wiggle. Keep it at a low priority so it
	 * never preempts the capsense TIMER0 ISR or USB; it only sets a flag. */
	systick_set_frequency(SYSTICK_FREQUENCY, AHB_FREQUENCY);
	systick_counter_enable();
	systick_interrupt_enable();
	nvic_set_priority(NVIC_SYSTICK_IRQ, 0xff);

	while (1) {
		/* Wait for a fresh capsense sweep. (== so it survives overflow.) */
		while (g_capsense_generation == last_generation)
			;
		last_generation = g_capsense_generation;

		/* Sum the channels; a touch pushes the total above the threshold. */
		uint32_t sum = 0;
		for (int i = 0; i < 4; i++)
			sum += g_channel_values[i];
		bool touched = (sum > CAPSENSE_DETECT_MIN);

		/* Count touched generations up to the threshold. Isolated misses (the
		 * reading flickering across the threshold mid-press) don't reset the
		 * count; only TOUCH_RELEASE_GENS consecutive idle generations -- a real
		 * lift-off -- do. So the count tracks actual touch time, and a single
		 * press yields a single toggle. */
		if (touched) {
			idle_run = 0;
			if (touch_count < TOUCH_HOLD_GENS)
				touch_count++;
		} else if (++idle_run >= TOUCH_RELEASE_GENS) {
			touch_count = 0;
			toggled_this_hold = false;
		}

		if (touch_count >= TOUCH_HOLD_GENS && !toggled_this_hold) {
			g_armed = !g_armed;
			update_leds();
			toggled_this_hold = true;
		}

		/* Perform a wiggle if SysTick has asked for one. Doing the USB write
		 * here (rather than in the ISR) keeps all endpoint writes on one
		 * thread of control. */
		if (g_do_wiggle) {
			g_do_wiggle = false;
			if (g_usbd_is_connected && g_armed)
				do_wiggle();
		}
	}
}

/* ---- capsense plumbing (proven setup from captouch / coinflip) --------- */

static void ACMP_CapsenseChannelSet(uint32_t channel)
{
	g_current_channel = channel;

	if (channel == 0) {
		MMIO32(ACMP0_INPUTSEL) = (acmpResistor0 << _ACMP_INPUTSEL_CSRESSEL_SHIFT)
					| ACMP_INPUTSEL_CSRESEN
					| (false << _ACMP_INPUTSEL_LPREF_SHIFT)
					| (0x3f << _ACMP_INPUTSEL_VDDLEVEL_SHIFT)
					| ACMP_INPUTSEL_NEGSEL(ACMP_INPUTSEL_NEGSEL_CAPSENSE)
					| (channel << _ACMP_INPUTSEL_POSSEL_SHIFT);
	}
	else if (channel == 1) {
		MMIO32(ACMP0_INPUTSEL) = (acmpResistor0 << _ACMP_INPUTSEL_CSRESSEL_SHIFT)
					| ACMP_INPUTSEL_CSRESEN
					| (false << _ACMP_INPUTSEL_LPREF_SHIFT)
					| (0x3d << _ACMP_INPUTSEL_VDDLEVEL_SHIFT)
					| ACMP_INPUTSEL_NEGSEL(ACMP_INPUTSEL_NEGSEL_CAPSENSE)
					| (channel << _ACMP_INPUTSEL_POSSEL_SHIFT);
	}
	else if (channel == 2)
		;
	else if (channel == 3)
		;
	else
		while(1);
}

static void CAPSENSE_Measure(uint32_t channel)
{
	/* Set up this channel in the ACMP. */
	ACMP_CapsenseChannelSet(channel);

	/* Reset timers */
	TIMER0_CNT = 0;
	TIMER1_CNT = 0;

	/* Start timers */
	TIMER0_CMD = TIMER_CMD_START;
	TIMER1_CMD = TIMER_CMD_START;

	if (channel == 2) {
		gpio_mode_setup(CAP0B_PORT, GPIO_MODE_PUSH_PULL, CAP0B_PIN);
		gpio_set(CAP0B_PORT, CAP0B_PIN);
		gpio_mode_setup(CAP0B_PORT, GPIO_MODE_INPUT, CAP0B_PIN);
		while (gpio_get(CAP0B_PORT, CAP0B_PIN) && (TIMER0_CNT < (TIMER0_TOP - 5)))
			;
		g_channel_values[channel] = TIMER0_CNT;
	}
	else if (channel == 3) {
		gpio_mode_setup(CAP1B_PORT, GPIO_MODE_PUSH_PULL, CAP1B_PIN);
		gpio_set(CAP1B_PORT, CAP1B_PIN);
		gpio_mode_setup(CAP1B_PORT, GPIO_MODE_INPUT, CAP1B_PIN);
		while (gpio_get(CAP1B_PORT, CAP1B_PIN) && (TIMER0_CNT < (TIMER0_TOP - 5)))
			;
		g_channel_values[channel] = TIMER0_CNT;
	}
}

void timer0_isr(void)
{
	uint32_t count;

	/* Stop timers */
	TIMER0_CMD = TIMER_CMD_STOP;
	TIMER1_CMD = TIMER_CMD_STOP;

	/* Clear interrupt flag */
	TIMER0_IFC = TIMER_IFC_OF;

	/* Read out value of TIMER1 */
	count = TIMER1_CNT;

	/* Store value in channelValues */
	g_channel_values[g_current_channel] = count;

	/* Update channelMaxValues */
	if (count > channelMaxValues[g_current_channel])
		channelMaxValues[g_current_channel] = count;

	if (g_capsense_running) {
		if (g_current_channel >= 3) {
			g_capsense_generation++;
			g_current_channel = 0;
		}
		else {
			g_current_channel++;
		}
		CAPSENSE_Measure(g_current_channel);
	}
	else {
		/* Disable the ACMP, since capsense is no longer running */
		MMIO32(ACMP0_CTRL) &= ~ACMP_CTRL_EN;
	}
}

void capsense_start(void)
{
	g_capsense_running = true;

	/* Set the "Enable" Bit in ACMP, so we can make analog measurements */
	MMIO32(ACMP0_CTRL) |= ACMP_CTRL_EN;

	CAPSENSE_Measure(0);
}

void capsense_stop(void)
{
	g_capsense_running = false;
}

void setup_acmp_capsense(const struct acmp_capsense_init *init)
{
	/* Make sure the module exists on the selected chip */
	EFM_ASSERT(ACMP_REF_VALID(acmp));

	/* Make sure that vddLevel is within bounds */
	EFM_ASSERT(init->vddLevel < 64);

	/* Make sure biasprog is within bounds */
	EFM_ASSERT(init->biasProg <=
			   (_ACMP_CTRL_BIASPROG_MASK >> _ACMP_CTRL_BIASPROG_SHIFT));

	/* Set control register. No need to set interrupt modes */
	MMIO32(ACMP0_CTRL) = (init->fullBias << _ACMP_CTRL_FULLBIAS_SHIFT)
					| (init->halfBias << _ACMP_CTRL_HALFBIAS_SHIFT)
					| (init->biasProg << _ACMP_CTRL_BIASPROG_SHIFT)
					| (init->warmTime << _ACMP_CTRL_WARMTIME_SHIFT)
					| (init->hysteresisLevel << _ACMP_CTRL_HYSTSEL_SHIFT)
		;

	/* Select capacative sensing mode by selecting a resistor and enabling it */
	MMIO32(ACMP0_INPUTSEL) = (init->resistor << _ACMP_INPUTSEL_CSRESSEL_SHIFT)
					| ACMP_INPUTSEL_CSRESEN
					| (init->lowPowerReferenceEnabled << _ACMP_INPUTSEL_LPREF_SHIFT)
					| (init->vddLevel << _ACMP_INPUTSEL_VDDLEVEL_SHIFT)
					| ACMP_INPUTSEL_NEGSEL(ACMP_INPUTSEL_NEGSEL_CAPSENSE)
		;

	/* Enable ACMP if requested. */
	if (init->enable)
		MMIO32(ACMP0_CTRL) |= (1 << _ACMP_CTRL_EN_SHIFT);
}

static void setup_capsense(void)
{
	const struct acmp_capsense_init capsenseInit = ACMP_CAPSENSE_INIT_DEFAULT;
	CMU_HFPERCLKDIV |= CMU_HFPERCLKDIV_HFPERCLKEN;
	cmu_periph_clock_enable(CMU_TIMER0);
	cmu_periph_clock_enable(CMU_TIMER1);

	CMU_HFPERCLKEN0 |= ACMP_CAPSENSE_CLKEN;
	cmu_periph_clock_enable(CMU_PRS);

	/* Initialize TIMER0 - Prescaler 2^9, top value 10, interrupt on overflow */
	TIMER0_CTRL = TIMER_CTRL_PRESC(TIMER_CTRL_PRESC_DIV512);
	TIMER0_TOP = 10;
	TIMER0_IEN = TIMER_IEN_OF;
	TIMER0_CNT = 0;

	/* Initialize TIMER1 - Prescaler 2^10, clock source CC1, top value 0xFFFF */
	TIMER1_CTRL = TIMER_CTRL_PRESC(TIMER_CTRL_PRESC_DIV1024) | TIMER_CTRL_CLKSEL(TIMER_CTRL_CLKSEL_CC1);
	TIMER1_TOP  = 0xFFFF;

	/* Set up TIMER1 CC1 to trigger on PRS channel 0 */
	TIMER1_CC1_CTRL = TIMER_CC_CTRL_MODE(TIMER_CC_CTRL_MODE_INPUTCAPTURE) /* Input capture      */
					| TIMER_CC_CTRL_PRSSEL(TIMER_CC_CTRL_PRSSEL_PRSCH0)   /* PRS channel 0      */
					| TIMER_CC_CTRL_INSEL           /* PRS input selected */
					| TIMER_CC_CTRL_ICEVCTRL(TIMER_CC_CTRL_ICEVCTRL_RISING) /* PRS on rising edge */
					| TIMER_CC_CTRL_ICEDGE(TIMER_CC_CTRL_ICEDGE_BOTH);    /* PRS on rising edge */

	/*Set up PRS channel 0 to trigger on ACMP0 output*/
	PRS_CH0_CTRL = PRS_CH_CTRL_EDSEL_POSEDGE              /* Posedge triggers action */
				   | PRS_CH_CTRL_SOURCESEL(PRS_CH_CTRL_SOURCESEL_ACMP_CAPSENSE)  /* PRS source */
				   | PRS_CH_CTRL_SIGSEL(PRS_CH_CTRL_SIGSEL_ACMPOUT_CAPSENSE); /* PRS signal */

	/* Set up ACMP0 in capsense mode */
	setup_acmp_capsense(&capsenseInit);

	/* Enable TIMER0 interrupt */
	nvic_enable_irq(NVIC_TIMER0_IRQ);
}

static void setup(void)
{
	/* GPIO peripheral clock is necessary for us to set up the GPIO pins as outputs */
	cmu_periph_clock_enable(CMU_GPIO);

	/* Set up both LEDs as outputs. Green uses the "drive" variant so it honors
	 * the reduced port-A drive strength set below (for dimming); red uses the
	 * plain mode and stays at standard drive (full brightness). */
	gpio_mode_setup(LED_RED_PORT, GPIO_MODE_WIRED_AND, LED_RED_PIN);
	gpio_mode_setup(LED_GREEN_PORT, GPIO_MODE_WIRED_AND_DRIVE, LED_GREEN_PIN);
	gpio_set_drive_strength(LED_GREEN_PORT, GREEN_DRIVE_STRENGTH);

	/* Disable GPIO for pin PC1 (CAP1A). The Toboot bootloader leaves it as an
	 * output, where it interferes with the analog comparator. */
	gpio_mode_setup(GPIOC, GPIO_MODE_DISABLE, GPIO1);

	setup_capsense();
}
