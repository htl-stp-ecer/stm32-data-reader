#include "wombat/core/ComponentVersionCheck.h"

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <string>
#include <unistd.h>

namespace
{
    // Cases from raccoon-transport contract/COMPONENT_VERSION_CHECK.md (own version v58).
    struct Case
    {
        const char* entry;  // JSON text of the entry
        const char* result;
        const char* reason;
    };

    class VersionCheckTest : public ::testing::Test
    {
    protected:
        void SetUp() override
        {
            char tmpl[] = "/tmp/reader_version_XXXXXX";
            dir_ = ::mkdtemp(tmpl);
            path_ = dir_ + "/bundle.json";
        }
        void TearDown() override
        {
            ::unlink(path_.c_str());
            ::rmdir(dir_.c_str());
        }
        void write(const std::string& text) { std::ofstream(path_) << text; }
        std::string dir_, path_;
    };
}

TEST_F(VersionCheckTest, Cases)
{
    const Case cases[] = {
        {"\"v58\"", "ok", ""},
        {"\"58\"", "ok", ""},
        {"\" v58 \"", "ok", ""},
        {"\"V58\"", "ok", ""},
        {"\"v59\"", "version-mismatch", ""},
        {"\"v058\"", "version-mismatch", ""},
        {"\"dev\"", "dev-warnung", "version-omitted"},
        {"\"\"", "bundle-fehler", "version-field-empty"},
        {"\"  \"", "bundle-fehler", "version-field-empty"},
        {"58", "bundle-fehler", "version-field-missing"},
    };
    for (const auto& c : cases)
    {
        write(std::string("{\"bundle\":\"b\",\"components\":{\"stm32-data-reader\":") + c.entry + "}}");
        const auto check = wombat::componentVersionCheck("v58", path_);
        ASSERT_TRUE(check.has_value()) << c.entry;
        EXPECT_EQ(check->result, c.result) << c.entry;
        EXPECT_EQ(check->reason, c.reason) << c.entry;
        EXPECT_EQ(check->name, "version");
    }
}

TEST_F(VersionCheckTest, MissingEntryAndMissingComponents)
{
    for (const char* text : {"{\"components\":{\"botui\":\"v1\"}}", "{\"contract_hash\":\"x\"}"})
    {
        write(text);
        const auto check = wombat::componentVersionCheck("58", path_);
        ASSERT_TRUE(check.has_value());
        EXPECT_EQ(check->result, "bundle-fehler");
        EXPECT_EQ(check->reason, "version-field-missing");
        EXPECT_NE(check->message.find("no version for stm32-data-reader"), std::string::npos);
    }
}

TEST_F(VersionCheckTest, MessagesNameBothSides)
{
    write(R"({"bundle":"2026.11.02","components":{"stm32-data-reader":"v59"}})");
    const auto check = wombat::componentVersionCheck("v58", path_);
    ASSERT_TRUE(check.has_value());
    EXPECT_EQ(check->message,
              "component version mismatch: stm32-data-reader is v58 but bundle 2026.11.02 expects v59; "
              "update or reflash so all components match");
    EXPECT_EQ(check->expected, "v59");
}

TEST_F(VersionCheckTest, NoReadableBundleMeansNoCheck)
{
    EXPECT_FALSE(wombat::componentVersionCheck("58", dir_ + "/missing.json").has_value());
    write("{nope");
    EXPECT_FALSE(wombat::componentVersionCheck("58", path_).has_value());
    write("[1]");
    EXPECT_FALSE(wombat::componentVersionCheck("58", path_).has_value());
}
