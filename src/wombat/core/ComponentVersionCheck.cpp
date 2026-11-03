// JSON parsing uses nlohmann/json (MIT), taken from the raccoon-transport submodule where it is
// vendored with its license (cpp/third_party/nlohmann). See THIRD_PARTY_NOTICES.md.
#include "wombat/core/ComponentVersionCheck.h"

#include <nlohmann/json.hpp>

#include <cstdlib>
#include <fstream>
#include <sstream>

namespace wombat
{
    namespace
    {
        constexpr const char* kBundleKey = "stm32-data-reader";
        constexpr const char* kComponent = "stm32-data-reader";
        constexpr const char* kDefaultBundleFile = "/etc/raccoon-bundle.json";

        std::string trim(const std::string& text)
        {
            const char* ws = " \t\r\n\f\v";
            const auto first = text.find_first_not_of(ws);
            if (first == std::string::npos) return {};
            return text.substr(first, text.find_last_not_of(ws) - first + 1);
        }

        // `v173` and `173` are the same version; nothing else is normalised.
        std::string normalize(const std::string& value)
        {
            const std::string t = trim(value);
            return (!t.empty() && (t[0] == 'v' || t[0] == 'V')) ? t.substr(1) : t;
        }

        std::string orUnknown(const std::string& value)
        {
            return value.empty() ? "unknown" : value;
        }
    }

    std::optional<raccoon::contract::ComponentCheck> componentVersionCheck(
        const std::string& componentVersion, const std::string& bundleFile)
    {
        std::string path = bundleFile;
        if (path.empty())
        {
            const char* env = std::getenv("RACCOON_BUNDLE_FILE");
            path = (env != nullptr && *env != '\0') ? env : kDefaultBundleFile;
        }

        std::ifstream in(path, std::ios::binary);
        if (!in.is_open()) return std::nullopt;
        std::ostringstream buffer;
        buffer << in.rdbuf();
        const auto document = nlohmann::json::parse(buffer.str(), nullptr, /*allow_exceptions=*/false);
        if (document.is_discarded() || !document.is_object()) return std::nullopt;

        std::string bundle;
        if (auto it = document.find("bundle"); it != document.end() && it->is_string()) bundle = it->get<std::string>();

        raccoon::contract::ComponentCheck check;
        check.name = "version";
        const auto components = document.find("components");
        const nlohmann::json* entry = nullptr;
        if (components != document.end() && components->is_object())
        {
            if (auto it = components->find(kBundleKey); it != components->end() && it->is_string()) entry = &*it;
        }

        if (entry == nullptr)
        {
            check.result = "bundle-fehler";
            check.reason = "version-field-missing";
            check.message = std::string("bundle check failed: bundle file has no version for ") + kBundleKey + ": " + path;
            return check;
        }
        const std::string expected = trim(entry->get<std::string>());
        if (expected.empty())
        {
            check.result = "bundle-fehler";
            check.reason = "version-field-empty";
            check.message = std::string("bundle check failed: bundle file has an empty version for ") + kBundleKey + ": " + path;
            return check;
        }
        check.expected = expected;
        if (expected == "dev")
        {
            check.result = "dev-warnung";
            check.reason = "version-omitted";
            check.message = std::string("component version check omitted ('") + kBundleKey +
                            "' is 'dev' in the bundle): " + kComponent + " " + orUnknown(componentVersion) +
                            " is not compared; mismatches are not enforced";
            return check;
        }
        if (normalize(expected) != normalize(componentVersion))
        {
            check.result = "version-mismatch";
            check.message = std::string("component version mismatch: ") + kComponent + " is " +
                            orUnknown(componentVersion) + " but bundle " + orUnknown(bundle) + " expects " + expected +
                            "; update or reflash so all components match";
            return check;
        }
        check.result = "ok";
        check.message = "version ok: " + componentVersion;
        return check;
    }
}
