#pragma once

#include <raccoon/ContractCheck.h>

#include <optional>
#include <string>

namespace wombat
{
    // The reader's own version against its "stm32-data-reader" entry in the bundle file
    // (/etc/raccoon-bundle.json). Specified for all components in raccoon-transport,
    // contract/COMPONENT_VERSION_CHECK.md; the transport's startup check only merges the
    // result (it is handed over as a component check).
    //
    // Returns nullopt when the bundle file cannot be read: the transport check reports that.
    std::optional<raccoon::contract::ComponentCheck> componentVersionCheck(
        const std::string& componentVersion, const std::string& bundleFile = {});
}
