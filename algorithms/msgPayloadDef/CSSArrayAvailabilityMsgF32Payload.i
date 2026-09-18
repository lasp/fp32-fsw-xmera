%module {type}Payload
%{{
    #include "{baseDir}/{type}Payload.h"
    #include <architecture/messaging/messaging.h>
    #include "utilities/fsw/deviceAvailability.h"
    #include <stdint.h>
%}}
%include <architecture/messaging/newMessaging.ih>

%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%template(TimeVector) std::vector<unsigned long long, std::allocator<unsigned long long>>;

%include <mission/parameters.h>

// Exposes DEVICE_AVAILABLE and DEVICE_UNAVAILABLE to Python, so a test names the state
// instead of repeating the integer.
%include "utilities/fsw/deviceAvailability.h"

// Lets Python assign a sequence to the enum array, as xmera does for its own availability messages.
ARRAYINTASLIST(DeviceAvailability_c)

%include <architecture/messaging/messaging.h>
%include <architecture/_GeneralModuleFiles/sys_model.h>

%rename(__subscribe_to) subscribeTo;
%rename(__is_subscribed_to) isSubscribedTo;
%rename(__time_vector) times;
%rename(__timeWritten_vector) timesWritten;
%rename(__record_vector) record;

%include "{baseDir}/{type}Payload.h"
INSTANTIATE_TEMPLATES({type}, {type}Payload, {baseDir})
