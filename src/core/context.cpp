#include "context.hpp"

namespace mz
{

Context::Context(ContextCreateInfo &context_cinfo) :
    instance(context_cinfo.app_name, context_cinfo.window),
    physical_device(instance.pick_physical_device(context_cinfo.device_extensions, context_cinfo.requested_features)),
    device(instance, physical_device, context_cinfo.device_extensions){};

}        // namespace mz