#include "button_map.h"
#include "usb_device_drivers.h"
#include "usb.h"
#include "utils.h"
#include "wiimote.h"

/* Nintendo GameCube Controller Adapter (WUP-028) and clones in native mode
 * (e.g. Mayflash set to "Wii U"), VID 057e PID 0337.
 *
 * One interrupt OUT byte 0x13 starts the polling, then the adapter sends
 * 37-byte interrupt IN reports: 0x21 followed by 9 bytes per port:
 *   [status, buttons1, buttons2, stick_x, stick_y, cstick_x, cstick_y, l, r]
 * status bit 4 (0x10) = wired controller, bit 5 (0x20) = wireless.
 * Sticks and triggers are 0-255 with sticks centered at ~128 and up/right
 * as high values. Rumble: interrupt OUT [0x11, p1, p2, p3, p4].
 *
 * The first connected port is used. */

#define GCA_REPORT_SIZE 37
#define GCA_NUM_PORTS 4
#define GCA_POLL_CMD 0x13
#define GCA_RUMBLE_CMD 0x11

enum gca_buttons_e {
	/* buttons1 */
	GCA_BUTTON_A,
	GCA_BUTTON_B,
	GCA_BUTTON_X,
	GCA_BUTTON_Y,
	GCA_BUTTON_LEFT,
	GCA_BUTTON_RIGHT,
	GCA_BUTTON_DOWN,
	GCA_BUTTON_UP,
	/* buttons2 */
	GCA_BUTTON_START,
	GCA_BUTTON_Z,
	GCA_BUTTON_R,
	GCA_BUTTON_L,
	/* synthesized from Z+Start */
	GCA_BUTTON_HOME,
	GCA_BUTTON__NUM
};

enum gca_analog_axis_e {
	GCA_ANALOG_AXIS_LEFT_X,
	GCA_ANALOG_AXIS_LEFT_Y,
	GCA_ANALOG_AXIS_RIGHT_X,
	GCA_ANALOG_AXIS_RIGHT_Y,
	GCA_ANALOG_AXIS__NUM
};

struct gca_private_data_t {
	struct {
		u32 buttons;
		u8 analog_axis[GCA_ANALOG_AXIS__NUM];
	} input;
	enum bm_ir_emulation_mode_e ir_emu_mode;
	struct bm_ir_emulation_state_t ir_emu_state;
	u8 mapping;
	u8 ir_emu_mode_idx;
	u8 active_port;
	bool rumble_on;
	bool hello_rumble_pending;
	bool switch_mapping;
	bool switch_ir_emu_mode;
};
static_assert(sizeof(struct gca_private_data_t) <= USB_INPUT_DEVICE_PRIVATE_DATA_SIZE);

/* Z+X+Y: switch between Wiimote+Nunchuk and Classic Controller mappings.
 * Z+B+X: cycle the IR pointer emulation mode. */
#define SWITCH_MAPPING_COMBO	 (BIT(GCA_BUTTON_Z) | BIT(GCA_BUTTON_X) | BIT(GCA_BUTTON_Y))
#define SWITCH_IR_EMU_MODE_COMBO (BIT(GCA_BUTTON_Z) | BIT(GCA_BUTTON_B) | BIT(GCA_BUTTON_X))

static const struct {
	enum wiimote_ext_e extension;
	u16 wiimote_button_map[GCA_BUTTON__NUM];
	u8 nunchuk_button_map[GCA_BUTTON__NUM];
	u8 nunchuk_analog_axis_map[GCA_ANALOG_AXIS__NUM];
	u16 classic_button_map[GCA_BUTTON__NUM];
	u8 classic_analog_axis_map[GCA_ANALOG_AXIS__NUM];
} input_mappings[] = {
	{
		.extension = WIIMOTE_EXT_NUNCHUK,
		.wiimote_button_map = {
			[GCA_BUTTON_A]     = WIIMOTE_BUTTON_A,
			[GCA_BUTTON_B]     = WIIMOTE_BUTTON_B,
			[GCA_BUTTON_X]     = WIIMOTE_BUTTON_TWO,
			[GCA_BUTTON_Y]     = WIIMOTE_BUTTON_ONE,
			[GCA_BUTTON_UP]    = WIIMOTE_BUTTON_UP,
			[GCA_BUTTON_DOWN]  = WIIMOTE_BUTTON_DOWN,
			[GCA_BUTTON_LEFT]  = WIIMOTE_BUTTON_LEFT,
			[GCA_BUTTON_RIGHT] = WIIMOTE_BUTTON_RIGHT,
			[GCA_BUTTON_START] = WIIMOTE_BUTTON_PLUS,
			[GCA_BUTTON_Z]     = WIIMOTE_BUTTON_MINUS,
			[GCA_BUTTON_HOME]  = WIIMOTE_BUTTON_HOME,
		},
		.nunchuk_button_map = {
			[GCA_BUTTON_L] = NUNCHUK_BUTTON_C,
			[GCA_BUTTON_R] = NUNCHUK_BUTTON_Z,
		},
		.nunchuk_analog_axis_map = {
			[GCA_ANALOG_AXIS_LEFT_X] = BM_NUNCHUK_ANALOG_AXIS_X,
			[GCA_ANALOG_AXIS_LEFT_Y] = BM_NUNCHUK_ANALOG_AXIS_Y,
		},
	},
	{
		.extension = WIIMOTE_EXT_CLASSIC,
		.classic_button_map = {
			[GCA_BUTTON_A]     = CLASSIC_CTRL_BUTTON_A,
			[GCA_BUTTON_B]     = CLASSIC_CTRL_BUTTON_B,
			[GCA_BUTTON_X]     = CLASSIC_CTRL_BUTTON_X,
			[GCA_BUTTON_Y]     = CLASSIC_CTRL_BUTTON_Y,
			[GCA_BUTTON_UP]    = CLASSIC_CTRL_BUTTON_UP,
			[GCA_BUTTON_DOWN]  = CLASSIC_CTRL_BUTTON_DOWN,
			[GCA_BUTTON_LEFT]  = CLASSIC_CTRL_BUTTON_LEFT,
			[GCA_BUTTON_RIGHT] = CLASSIC_CTRL_BUTTON_RIGHT,
			[GCA_BUTTON_START] = CLASSIC_CTRL_BUTTON_PLUS,
			[GCA_BUTTON_Z]     = CLASSIC_CTRL_BUTTON_ZR,
			[GCA_BUTTON_R]     = CLASSIC_CTRL_BUTTON_FULL_R,
			[GCA_BUTTON_L]     = CLASSIC_CTRL_BUTTON_FULL_L,
			[GCA_BUTTON_HOME]  = CLASSIC_CTRL_BUTTON_HOME,
		},
		.classic_analog_axis_map = {
			[GCA_ANALOG_AXIS_LEFT_X]  = BM_CLASSIC_ANALOG_AXIS_LEFT_X,
			[GCA_ANALOG_AXIS_LEFT_Y]  = BM_CLASSIC_ANALOG_AXIS_LEFT_Y,
			[GCA_ANALOG_AXIS_RIGHT_X] = BM_CLASSIC_ANALOG_AXIS_RIGHT_X,
			[GCA_ANALOG_AXIS_RIGHT_Y] = BM_CLASSIC_ANALOG_AXIS_RIGHT_Y,
		},
	},
};

static const u8 ir_analog_axis_map[GCA_ANALOG_AXIS__NUM] = {
	[GCA_ANALOG_AXIS_RIGHT_X] = BM_IR_AXIS_X,
	[GCA_ANALOG_AXIS_RIGHT_Y] = BM_IR_AXIS_Y,
};

static const enum bm_ir_emulation_mode_e ir_emu_modes[] = {
	BM_IR_EMULATION_MODE_ABSOLUTE_ANALOG_AXIS,
	BM_IR_EMULATION_MODE_RELATIVE_ANALOG_AXIS,
	BM_IR_EMULATION_MODE_NONE,
};

static inline int gca_request_data(usb_input_device_t *device)
{
	/* Request exactly one report: the interrupt endpoint's max packet size
	 * is 37 bytes and larger requests fail or overflow. */
	return usb_device_driver_issue_intr_transfer_async(device, 0, device->usb_async_resp,
							   GCA_REPORT_SIZE);
}

static int gca_send_rumble(usb_input_device_t *device)
{
	struct gca_private_data_t *priv = (void *)device->private_data;
	u8 buf[5] ATTRIBUTE_ALIGN(32) = {GCA_RUMBLE_CMD, 0, 0, 0, 0};

	buf[1 + priv->active_port] = priv->rumble_on ? 1 : 0;

	return usb_device_driver_issue_intr_transfer(device, 1, buf, sizeof(buf));
}

bool gca_driver_ops_probe(u16 vid, u16 pid)
{
	static const struct device_id_t compatible[] = {
		{0x057e, 0x0337},
	};

	return usb_driver_is_comaptible(vid, pid, compatible, ARRAY_SIZE(compatible));
}

int gca_driver_ops_init(usb_input_device_t *device, u16 vid, u16 pid)
{
	int ret;
	struct gca_private_data_t *priv = (void *)device->private_data;
	u8 poll_cmd[1] ATTRIBUTE_ALIGN(32) = {GCA_POLL_CMD};

	/* Init private state */
	priv->ir_emu_mode_idx = 0;
	bm_ir_emulation_state_reset(&priv->ir_emu_state);
	priv->mapping = 0;
	priv->active_port = 0;
	priv->rumble_on = false;
	priv->switch_mapping = false;
	priv->switch_ir_emu_mode = false;
	priv->input.buttons = 0;
	priv->input.analog_axis[GCA_ANALOG_AXIS_LEFT_X] = 128;
	priv->input.analog_axis[GCA_ANALOG_AXIS_LEFT_Y] = 128;
	priv->input.analog_axis[GCA_ANALOG_AXIS_RIGHT_X] = 128;
	priv->input.analog_axis[GCA_ANALOG_AXIS_RIGHT_Y] = 128;

	/* Set initial extension */
	fake_wiimote_set_extension(device->wiimote, input_mappings[priv->mapping].extension);

	/* Start the adapter's polling */
	ret = usb_device_driver_issue_intr_transfer(device, 1, poll_cmd, sizeof(poll_cmd));
	if (ret < 0)
		return ret;

	/* Proof-of-life: buzz all ports briefly, stopped on the first report */
	{
		u8 hello[5] ATTRIBUTE_ALIGN(32) = {GCA_RUMBLE_CMD, 1, 1, 1, 1};
		usb_device_driver_issue_intr_transfer(device, 1, hello, sizeof(hello));
		priv->hello_rumble_pending = true;
	}

	ret = gca_request_data(device);
	if (ret < 0)
		return ret;

	return 0;
}

int gca_driver_ops_disconnect(usb_input_device_t *device)
{
	struct gca_private_data_t *priv = (void *)device->private_data;

	priv->rumble_on = false;

	return gca_send_rumble(device);
}

int gca_driver_ops_slot_changed(usb_input_device_t *device, u8 slot)
{
	/* The adapter has no player LEDs */
	return 0;
}

int gca_driver_ops_set_rumble(usb_input_device_t *device, bool rumble_on)
{
	struct gca_private_data_t *priv = (void *)device->private_data;

	priv->rumble_on = rumble_on;

	return gca_send_rumble(device);
}

bool gca_report_input(usb_input_device_t *device)
{
	struct gca_private_data_t *priv = (void *)device->private_data;
	u16 wiimote_buttons = 0;
	union wiimote_extension_data_t extension_data;
	struct ir_dot_t ir_dots[IR_MAX_DOTS];
	enum bm_ir_emulation_mode_e ir_emu_mode;

	if (bm_check_switch_mapping(priv->input.buttons, &priv->switch_mapping, SWITCH_MAPPING_COMBO)) {
		priv->mapping = (priv->mapping + 1) % ARRAY_SIZE(input_mappings);
		fake_wiimote_set_extension(device->wiimote, input_mappings[priv->mapping].extension);
		return false;
	} else if (bm_check_switch_mapping(priv->input.buttons, &priv->switch_ir_emu_mode, SWITCH_IR_EMU_MODE_COMBO)) {
		priv->ir_emu_mode_idx = (priv->ir_emu_mode_idx + 1) % ARRAY_SIZE(ir_emu_modes);
		bm_ir_emulation_state_reset(&priv->ir_emu_state);
	}

	bm_map_wiimote(GCA_BUTTON__NUM, priv->input.buttons,
		       input_mappings[priv->mapping].wiimote_button_map,
		       &wiimote_buttons);

	ir_emu_mode = ir_emu_modes[priv->ir_emu_mode_idx];
	if (ir_emu_mode == BM_IR_EMULATION_MODE_NONE) {
		bm_ir_dots_set_out_of_screen(ir_dots);
	} else {
		bm_map_ir_analog_axis(ir_emu_mode, &priv->ir_emu_state,
				      GCA_ANALOG_AXIS__NUM, priv->input.analog_axis,
				      ir_analog_axis_map, ir_dots);
	}

	fake_wiimote_report_ir_dots(device->wiimote, ir_dots);

	if (input_mappings[priv->mapping].extension == WIIMOTE_EXT_NONE) {
		fake_wiimote_report_input(device->wiimote, wiimote_buttons);
	} else if (input_mappings[priv->mapping].extension == WIIMOTE_EXT_NUNCHUK) {
		bm_map_nunchuk(GCA_BUTTON__NUM, priv->input.buttons,
			       GCA_ANALOG_AXIS__NUM, priv->input.analog_axis,
			       0, 0, 0,
			       input_mappings[priv->mapping].nunchuk_button_map,
			       input_mappings[priv->mapping].nunchuk_analog_axis_map,
			       &extension_data.nunchuk);
		fake_wiimote_report_input_ext(device->wiimote, wiimote_buttons,
					      &extension_data, sizeof(extension_data.nunchuk));
	} else if (input_mappings[priv->mapping].extension == WIIMOTE_EXT_CLASSIC) {
		bm_map_classic(GCA_BUTTON__NUM, priv->input.buttons,
			       GCA_ANALOG_AXIS__NUM, priv->input.analog_axis,
			       input_mappings[priv->mapping].classic_button_map,
			       input_mappings[priv->mapping].classic_analog_axis_map,
			       &extension_data.classic);
		fake_wiimote_report_input_ext(device->wiimote, wiimote_buttons,
					      &extension_data, sizeof(extension_data.classic));
	}

	return true;
}

int gca_driver_ops_usb_async_resp(usb_input_device_t *device)
{
	struct gca_private_data_t *priv = (void *)device->private_data;
	const u8 *report = device->usb_async_resp;
	const u8 *pad = NULL;
	u32 buttons = 0;

	if (report[0] == 0x21) {
		/* use the first port with a controller plugged in */
		for (int port = 0; port < GCA_NUM_PORTS; port++) {
			const u8 *data = &report[1 + 9 * port];
			if (data[0] & 0x30) {
				pad = data;
				priv->active_port = port;
				break;
			}
		}
	}

	if (priv->hello_rumble_pending) {
		static int hello_reports = 0;
		if (++hello_reports > 100) {
			u8 quiet[5] ATTRIBUTE_ALIGN(32) = {GCA_RUMBLE_CMD, 0, 0, 0, 0};
			usb_device_driver_issue_intr_transfer(device, 1, quiet, sizeof(quiet));
			priv->hello_rumble_pending = false;
		}
	}

	if (pad) {
		buttons = pad[1] | (pad[2] << 8);

		/* Z+Start = HOME */
		if ((buttons & (BIT(GCA_BUTTON_Z) | BIT(GCA_BUTTON_START))) ==
		    (BIT(GCA_BUTTON_Z) | BIT(GCA_BUTTON_START))) {
			buttons &= ~(BIT(GCA_BUTTON_Z) | BIT(GCA_BUTTON_START));
			buttons |= BIT(GCA_BUTTON_HOME);
		}

		priv->input.buttons = buttons;
		priv->input.analog_axis[GCA_ANALOG_AXIS_LEFT_X] = pad[3];
		priv->input.analog_axis[GCA_ANALOG_AXIS_LEFT_Y] = pad[4];
		priv->input.analog_axis[GCA_ANALOG_AXIS_RIGHT_X] = pad[5];
		priv->input.analog_axis[GCA_ANALOG_AXIS_RIGHT_Y] = pad[6];
	} else {
		priv->input.buttons = 0;
		priv->input.analog_axis[GCA_ANALOG_AXIS_LEFT_X] = 128;
		priv->input.analog_axis[GCA_ANALOG_AXIS_LEFT_Y] = 128;
		priv->input.analog_axis[GCA_ANALOG_AXIS_RIGHT_X] = 128;
		priv->input.analog_axis[GCA_ANALOG_AXIS_RIGHT_Y] = 128;
	}

	return gca_request_data(device);
}

const usb_device_driver_t gca_usb_device_driver = {
	.probe		= gca_driver_ops_probe,
	.init		= gca_driver_ops_init,
	.disconnect	= gca_driver_ops_disconnect,
	.slot_changed	= gca_driver_ops_slot_changed,
	.set_rumble	= gca_driver_ops_set_rumble,
	.report_input	= gca_report_input,
	.usb_async_resp	= gca_driver_ops_usb_async_resp,
};
