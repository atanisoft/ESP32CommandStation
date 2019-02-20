#ifndef _OPENLCB_SERVOCONSUMERCONFIG_HXX_
#define _OPENLCB_SERVOCONSUMERCONFIG_HXX_

#include "openlcb/ConfigRepresentation.hxx"

namespace openlcb
{

CDI_GROUP(ServoConsumerConfig);
CDI_GROUP_ENTRY(description, StringConfigEntry<16>, Name("Description"),
    Description("User name of this output."));
CDI_GROUP_ENTRY(event_rotate_min, EventConfigEntry, //
    Name("Minimum Rotation Event ID"),
    Description("Receiving this event ID will rotate the servo to its mimimum "
                "configured point."));
CDI_GROUP_ENTRY(event_rotate_max, EventConfigEntry, //
    Name("Maximum Rotation Event ID"),
    Description("Receiving this event ID will rotate the servo to its maximum "
                "configured point."));
#define SERVO_DESCRIPTION_SUFFIX                                               \
    "stop point of the servo, as a percentage: generally 0-100. "              \
    "May be under/over-driven by setting a percentage value "                  \
    "of -99 to 200, respectively."
CDI_GROUP_ENTRY(servo_min_percent, Int16ConfigEntry, Default(0), Min(-99),
    Max(200), Name("Servo Minimum Stop Point Percentage"),
    Description("Low-end " SERVO_DESCRIPTION_SUFFIX));
CDI_GROUP_ENTRY(servo_max_percent, Int16ConfigEntry, Default(100), Min(-99),
    Max(200), Name("Servo Maximum Stop Point Percentage"),
    Description("High-end " SERVO_DESCRIPTION_SUFFIX));
CDI_GROUP_END(); // ServoConsumerConfig

} // namespace openlcb

#endif // _OPENLCB_SERVOCONSUMERCONFIG_HXX_