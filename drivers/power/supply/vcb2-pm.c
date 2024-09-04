/*
* Viviana Cloud Box 2 Power Management Driver
*/

#include <linux/module.h>
#include <linux/device.h>
#include <linux/serdev.h>
#include <linux/power_supply.h>
#include <linux/of_device.h>
#include <linux/reboot.h>
#include <linux/mutex.h>
#include <linux/wait.h>

#define PM_STATUS_BATTERY_LEVEL_BITS 0x7f
#define PM_STATUS_ONLINE_BITS (1 << 7)
#define PM_STATUS_CACHE_PERIOD_MS 100

#define SERIAL_BAUDRATE 115200
#define SERIAL_TIMEOUT_MS 1000

#define GRACEFUL_SHUTDOWN_MS 20000

struct vcb2_pm_status_parser {
	enum {
		PARSER_STATE_INIT,
		PARSER_STATE_SHUTDOWN_CONFIRM,
		PARSER_STATE_PING_CONFIRM,
		PARSER_STATE_COMMA,
		PARSER_STATE_PERCENT,
	} state;

	u8 status;
};

static void vcb2_pm_status_parser_init(struct vcb2_pm_status_parser *p)
{
	p->state = PARSER_STATE_INIT;
	p->status = 0;
}

enum vcb2_pm_status_parser_ret {
	PARSER_RET_COMPLETE_STATUS,
	PARSER_RET_COMPLETE_SHUTDOWN_REQUEST,
	PARSER_RET_COMPLETE_PING,
	PARSER_RET_INCOMPLETE,
	PARSER_RET_ERROR,
};

/*
 * Simple state machine parser for the serial messages from PM.
 * 
 * Gramma (PEG):
 * message <- (power_status / shutdown_request / ping) '\n'
 * power_status <- ext_power_online ',' bat_percent
 * shutdown_request <- 's'
 * ping <- 'p'
 * ext_power_online <- '0' / '1'
 * bat_percent <- [0-9]*
 * 
 * Percent values are not validated. Values outside of the allowed
 * range (0-100) are subject to unsigned integer overflow.
 * Empty percent value is treated as 0.
 */
static enum vcb2_pm_status_parser_ret
vcb2_pm_status_parser_feed(struct vcb2_pm_status_parser *p, const char c)
{
	u8 percent;

	switch (p->state) {
	case PARSER_STATE_INIT:
		switch (c) {
		case '0':
			p->status = 0;
			p->state = PARSER_STATE_COMMA;
			break;
		case '1':
			p->status = PM_STATUS_ONLINE_BITS;
			p->state = PARSER_STATE_COMMA;
			break;
		case 's':
			p->state = PARSER_STATE_SHUTDOWN_CONFIRM;
			break;
		case 'p':
			p->state = PARSER_STATE_PING_CONFIRM;
			break;
		default:
			goto err;
		}
		break;
	case PARSER_STATE_SHUTDOWN_CONFIRM:
		if (c == '\n') {
			p->state = PARSER_STATE_INIT;
			return PARSER_RET_COMPLETE_SHUTDOWN_REQUEST;
		} else {
			goto err;
		}
	case PARSER_STATE_PING_CONFIRM:
		if (c == '\n') {
			p->state = PARSER_STATE_INIT;
			return PARSER_RET_COMPLETE_PING;
		} else {
			goto err;
		}
	case PARSER_STATE_COMMA:
		if (c == ',')
			p->state = PARSER_STATE_PERCENT;
		else
			goto err;
		break;
	case PARSER_STATE_PERCENT:
		if (c >= '0' && c <= '9') {
			percent = p->status & PM_STATUS_BATTERY_LEVEL_BITS;
			percent = (percent * 10) + (u8)(c - '0');
			p->status =
				(p->status & ~PM_STATUS_BATTERY_LEVEL_BITS) |
				percent;
		} else if (c == '\n') {
			p->state = PARSER_STATE_INIT;
			return PARSER_RET_COMPLETE_STATUS;
		} else {
			goto err;
		}
		break;
	}

	return PARSER_RET_INCOMPLETE;

err:
	p->state = PARSER_STATE_INIT;
	return PARSER_RET_ERROR;
}

enum vcb2_pm_poll_state {
	POLL_STATE_IN_PROGRESS,
	POLL_STATE_COMPLETED,
	POLL_STATE_ERROR,
};

struct vcb2_pm_device_info {
	struct serdev_device *serdev;
	struct power_supply *psy;

	struct mutex lock;
	wait_queue_head_t wq; /* Wait queue for poll completion */

	u64 poll_time; /* Last time a poll was requested or received
                        * (depending on poll_state)
                        */

	struct timer_list graceful_shutdown_timer;

	enum vcb2_pm_poll_state poll_state;
	struct vcb2_pm_status_parser status_parser;

	u8 pm_status; /* [7]   = ext. power online flag
	               * [6:0] = battery level in percent
                       */

	bool shutdown_requested;
};

static void vcb2_pm_di_init(struct vcb2_pm_device_info *di)
{
	di->serdev = NULL;
	di->psy = NULL;
	di->poll_state = POLL_STATE_ERROR;
	di->poll_time = 0;
	di->pm_status = 0;
	di->shutdown_requested = false;

	mutex_init(&di->lock);
	init_waitqueue_head(&di->wq);
	vcb2_pm_status_parser_init(&di->status_parser);
}

static inline bool
vcb2_pm_is_poll_in_progress_unlocked(struct vcb2_pm_device_info *di)
{
	WARN_ON(!mutex_is_locked(&di->lock));

	return di->poll_state == POLL_STATE_IN_PROGRESS &&
	       time_is_after_jiffies64(di->poll_time +
				       msecs_to_jiffies(SERIAL_TIMEOUT_MS));
}

static inline bool
vcb2_pm_has_cached_status_unlocked(struct vcb2_pm_device_info *di)
{
	WARN_ON(!mutex_is_locked(&di->lock));

	return di->poll_state == POLL_STATE_COMPLETED &&
	       time_is_after_jiffies64(
		       di->poll_time +
		       msecs_to_jiffies(PM_STATUS_CACHE_PERIOD_MS));
}

static void run_userspace_poweroff(struct timer_list *timer)
{
	static char *shutdown_argv[] = { "/sbin/poweroff", NULL };
	printk("vcb2-pm - grace period ended; shutting down...\n");
	call_usermodehelper(shutdown_argv[0], shutdown_argv, NULL, UMH_NO_WAIT);
}

/*
 * Singals the shutdown request via sysfs attr "shutdown_requested".
 * If the system is still running after GRACEFUL_SHUTDOWN_MS a userspace
 * poweroff command is issued.
 */
static void initiate_shutdown_unlocked(struct vcb2_pm_device_info *di)
{
	WARN_ON(!mutex_is_locked(&di->lock));

	di->shutdown_requested = true;
	timer_setup(&di->graceful_shutdown_timer, run_userspace_poweroff, 0);
	mod_timer(&di->graceful_shutdown_timer,
		  jiffies + msecs_to_jiffies(GRACEFUL_SHUTDOWN_MS));
}

static int vcb2_pm_await_poll_response(struct vcb2_pm_device_info *di,
				       u8 *status)
{
	long ret;

	mutex_lock(&di->lock);
	while (di->poll_state == POLL_STATE_IN_PROGRESS) {
		mutex_unlock(&di->lock);
		ret = wait_event_interruptible_timeout(
			di->wq, di->poll_state != POLL_STATE_IN_PROGRESS,
			msecs_to_jiffies(SERIAL_TIMEOUT_MS));

		if (ret == 0)
			return -ETIMEDOUT;
		else if (ret < 0)
			return ret;

		mutex_lock(&di->lock);
	}

	if (di->poll_state == POLL_STATE_ERROR) {
		mutex_unlock(&di->lock);
		return -EIO;
	}

	*status = di->pm_status;
	mutex_unlock(&di->lock);
	return 0;
}

static int vcb2_pm_poll(struct vcb2_pm_device_info *di, u8 *status)
{
	static const char poll_msg[] = { 'p', 'o', 'l', 'l', '\n' };
	int ret;

	mutex_lock(&di->lock);
	if (vcb2_pm_has_cached_status_unlocked(di)) {
		*status = di->pm_status;
		mutex_unlock(&di->lock);
		return 0;
	} else if (vcb2_pm_is_poll_in_progress_unlocked(di)) {
		/* Poll already in progress; piggypack on it */
		mutex_unlock(&di->lock);
	} else {
		di->poll_state = POLL_STATE_IN_PROGRESS;
		di->poll_time = get_jiffies_64();
		mutex_unlock(&di->lock);

		ret = serdev_device_write(di->serdev, poll_msg,
					  sizeof(poll_msg),
					  msecs_to_jiffies(SERIAL_TIMEOUT_MS));
		if (ret < 0)
			return ret;

		if (ret < sizeof(poll_msg)) {
			/*
                         * This only happens when a singal or timeout was 
                         * received after some bytes were already written.
                         * This should never happen because only one poll
                         * message can be in flight at a time and we are only
                         * writing a very small buffer.
                         */
			return -EINTR;
		}
	}
	return vcb2_pm_await_poll_response(di, status);
}

static int vcb2_pm_serial_recv(struct serdev_device *serdev,
			       const unsigned char *buffer, size_t size)
{
	struct vcb2_pm_device_info *di = serdev_device_get_drvdata(serdev);
	enum vcb2_pm_status_parser_ret ret;
	bool should_wakup = false;

	for (int i = 0; i < size; i++) {
		ret = vcb2_pm_status_parser_feed(&di->status_parser, buffer[i]);
		switch (ret) {
		case PARSER_RET_INCOMPLETE:
			break;
		case PARSER_RET_ERROR:
			dev_err(&serdev->dev,
				"error while parsing PM status response\n");

			mutex_lock(&di->lock);
			di->poll_state = POLL_STATE_ERROR;
			mutex_unlock(&di->lock);

			should_wakup = true;
			break;
		case PARSER_RET_COMPLETE_STATUS:
			mutex_lock(&di->lock);
			di->poll_state = POLL_STATE_COMPLETED;
			di->pm_status = di->status_parser.status;
			di->poll_time = get_jiffies_64();
			mutex_unlock(&di->lock);

			should_wakup = true;
			break;
		case PARSER_RET_COMPLETE_SHUTDOWN_REQUEST:
			dev_info(&serdev->dev, "shutdown requested\n");
			mutex_lock(&di->lock);
			initiate_shutdown_unlocked(di);
			mutex_unlock(&di->lock);
			break;
		case PARSER_RET_COMPLETE_PING:
			static const char ping_msg[] = { 'p', '\n' };

			dev_info(&serdev->dev, "ping received\n");
			ret = serdev_device_write(
				di->serdev, ping_msg, sizeof(ping_msg),
				msecs_to_jiffies(SERIAL_TIMEOUT_MS));

			if (ret != sizeof(ping_msg))
				dev_err(&serdev->dev, "failed to send pong\n");

			break;
		}
	}

	if (should_wakup)
		wake_up_all(&di->wq);

	return size;
}

static int vcb2_pm_psy_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct vcb2_pm_device_info *di;
	int res;
	u8 pm_status;

	di = power_supply_get_drvdata(psy);
	res = vcb2_pm_poll(di, &pm_status);
	if (res)
		return res;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = !!(pm_status & PM_STATUS_ONLINE_BITS);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = pm_status & PM_STATUS_BATTERY_LEVEL_BITS;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static ssize_t show_shutdown_requested(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct vcb2_pm_device_info *di =
		power_supply_get_drvdata(to_power_supply(dev));
	return sysfs_emit(buf, "%d\n", di->shutdown_requested);
}

int vcb2_pm_sys_off_handler(struct sys_off_data *data)
{
	static const char shutdown_msg[] = { 'o', 'f', 'f', '\n' };
	struct serdev_device *serdev;
	int ret;

	serdev = data->cb_data;
	ret = serdev_device_write(serdev, shutdown_msg, sizeof(shutdown_msg),
				  msecs_to_jiffies(1000));

	if (ret != sizeof(shutdown_msg)) {
		dev_err(&serdev->dev, "error sending shutdown message\n");
	}

	serdev_device_wait_until_sent(serdev,
				      msecs_to_jiffies(SERIAL_TIMEOUT_MS));

	return NOTIFY_DONE;
}

static const struct serdev_device_ops vcb2_pm_ops = {
	.receive_buf = vcb2_pm_serial_recv,
	.write_wakeup = serdev_device_write_wakeup,
};

static int vcb2_pm_setup_serdev(struct vcb2_pm_device_info *di,
				struct serdev_device *serdev)
{
	int ret;
	uint baudrate;

	ret = serdev_device_open(serdev);
	if (ret) {
		dev_err(&serdev->dev, "error opening serial port\n");
		return -ret;
	}

	baudrate = serdev_device_set_baudrate(serdev, SERIAL_BAUDRATE);
	if (baudrate != SERIAL_BAUDRATE) {
		dev_err(&serdev->dev, "error setting baud rate\n");
		ret = -EINVAL;
		goto err;
	}

	ret = serdev_device_set_parity(serdev, SERDEV_PARITY_NONE);
	if (ret) {
		dev_err(&serdev->dev, "error setting parity\n");
		goto err;
	}

	serdev_device_set_flow_control(serdev, false);
	serdev_device_set_client_ops(serdev, &vcb2_pm_ops);

	di->serdev = serdev;
	dev_set_drvdata(&serdev->dev, di);
	return 0;

err:
	serdev_device_close(serdev);
	return ret;
}

static int vcb2_pm_setup_psy(struct vcb2_pm_device_info *di)
{
	/* sysfs attributes for custom properties */
	static struct device_attribute dev_attr_shutdown_requested =
		__ATTR(shutdown_requested, 0444, show_shutdown_requested, NULL);

	static struct attribute *vcb2_pm_sysfs_entries[] = {
		&dev_attr_shutdown_requested.attr,
		NULL,
	};

	static const struct attribute_group vcb2_pm_attr_group = {
		.name = NULL, /* put in device directory */
		.attrs = vcb2_pm_sysfs_entries,
	};

	static const struct attribute_group *vcb2_pm_attr_groups[] = {
		&vcb2_pm_attr_group,
		NULL,
	};

	/* Standard properties */
	static enum power_supply_property psy_props[] = {
		POWER_SUPPLY_PROP_ONLINE, POWER_SUPPLY_PROP_CAPACITY
	};

	struct power_supply_config psy_cfg = {
		.drv_data = di,
		.of_node = di->serdev->dev.of_node,
		.attr_grp = vcb2_pm_attr_groups,
	};

	struct power_supply_desc *psy_desc =
		devm_kzalloc(&di->serdev->dev, sizeof(*psy_desc), GFP_KERNEL);

	if (!psy_desc)
		return -ENOMEM;

	psy_desc->name = "vcb2-pm";
	psy_desc->type = POWER_SUPPLY_TYPE_BATTERY;
	psy_desc->properties = psy_props;
	psy_desc->num_properties = ARRAY_SIZE(psy_props);
	psy_desc->get_property = vcb2_pm_psy_get_property;

	di->psy = devm_power_supply_register(&di->serdev->dev, psy_desc,
					     &psy_cfg);

	if (IS_ERR(di->psy))
		return PTR_ERR(di->psy);

	return 0;
}

static int vcb2_pm_probe(struct serdev_device *serdev)
{
	struct vcb2_pm_device_info *di;
	int ret;

	di = devm_kzalloc(&serdev->dev, sizeof(*di), GFP_KERNEL);
	if (!di)
		return -ENOMEM;

	vcb2_pm_di_init(di);

	ret = vcb2_pm_setup_serdev(di, serdev);
	if (ret) {
		dev_err(&serdev->dev, "error setting up serial\n");
		return ret;
	}

	ret = vcb2_pm_setup_psy(di);
	if (ret) {
		dev_err(&di->psy->dev, "error registering power supply\n");
		goto err;
	}

	ret = devm_register_sys_off_handler(&serdev->dev,
					    SYS_OFF_MODE_POWER_OFF_PREPARE,
					    SYS_OFF_PRIO_DEFAULT,
					    vcb2_pm_sys_off_handler, serdev);

	if (ret) {
		dev_err(&serdev->dev, "error registering power off handler\n");
		goto err;
	}

	dev_info(&di->psy->dev, "power supply registered\n");
	return 0;

err:
	serdev_device_close(serdev);
	return ret;
}

static void vcb2_pm_remove(struct serdev_device *serdev)
{
	serdev_device_close(serdev);
}

static struct of_device_id vcb2_pm_ids[] = { { .compatible = "viv,vcb2-pm" },
					     {} };

MODULE_DEVICE_TABLE(of, vcb2_pm_ids);

static struct serdev_device_driver
	vcb2_pm_driver = { .probe = vcb2_pm_probe,
			   .remove = vcb2_pm_remove,
			   .driver = {
				   .name = "vcb2-pm",
				   .of_match_table = vcb2_pm_ids,
			   } };

static int __init vcb2_pm_mod_init(void)
{
	if (serdev_device_driver_register(&vcb2_pm_driver)) {
		printk("vcb2-pm - could not load driver\n");
		return -1;
	}
	return 0;
}

static void __exit vcb2_pm_mod_exit(void)
{
	serdev_device_driver_unregister(&vcb2_pm_driver);
}

module_init(vcb2_pm_mod_init);
module_exit(vcb2_pm_mod_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Strittmatter");
MODULE_DESCRIPTION("Viviana Cloud Box 2 Power Management Driver");
