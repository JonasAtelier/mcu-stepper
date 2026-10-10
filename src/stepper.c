#include <errno.h>

#include "stepper.h"

int stepper_enable(struct stepper_dev *stepper, bool enable)
{
	if (!stepper || !stepper->ops || !stepper->ops->enable)
		return -EINVAL;

	return stepper->ops->enable(stepper, enable);
}

int stepper_set_current(struct stepper_dev *stepper, float run_a,
			float hold_a)
{
	if (!stepper || !stepper->ops || !stepper->ops->set_current)
		return -EINVAL;

	return stepper->ops->set_current(stepper, run_a, hold_a);
}

int stepper_read_status(struct stepper_dev *stepper,
			struct stepper_status *status)
{
	struct stepper_status s;
	int ret;

	if (!stepper || !stepper->ops || !stepper->ops->read_status ||
	    !status)
		return -EINVAL;

	/* Into scratch first, so a failed read leaves *status alone. */
	ret = stepper->ops->read_status(stepper, &s);
	if (ret)
		return ret;
	*status = s;
	return 0;
}
