%module {type}Payload
%{{
    #include "{baseDir}/{type}Payload.h"
    #include <architecture/messaging/messaging.h>
    #include <stdint.h>
%}}
%include <architecture/messaging/newMessaging.ih>

%include <std_vector.i>
%include <architecture/_GeneralModuleFiles/swig_conly_data.i>

%template(TimeVector) std::vector<unsigned long long, std::allocator<unsigned long long>>;

%include <mission/parameters.h>

// Lets Python read and assign the uint16_t rowSums/colSums arrays as lists; SWIG's default array
// typemaps only cover int, unsigned int, float and double. xmera adds the same line to its shared template.
ARRAYINTASLIST(unsigned short)

%include <architecture/messaging/messaging.h>
%include <architecture/_GeneralModuleFiles/sys_model.h>

%rename(__subscribe_to) subscribeTo;
%rename(__is_subscribed_to) isSubscribedTo;
%rename(__time_vector) times;
%rename(__timeWritten_vector) timesWritten;
%rename(__record_vector) record;

%include "{baseDir}/{type}Payload.h"
INSTANTIATE_TEMPLATES({type}, {type}Payload, {baseDir})
