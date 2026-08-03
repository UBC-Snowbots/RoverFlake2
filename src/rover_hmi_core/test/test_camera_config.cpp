#include <gtest/gtest.h>
#include <rover_hmi_core/cameras/camera_config.h>

#include <QFile>

using namespace rover_hmi_core;

TEST(CameraConfig, ParsesValidJson) {
    QString err;
    auto cams = camera_config::parse(
        R"({"cameras":[{"name":"logitech","label":"Rear"},
                       {"name":"suyin","label":"Mast"}]})", &err);
    ASSERT_EQ(cams.size(), 2u) << err.toStdString();
    EXPECT_EQ(cams[0].name, "logitech");
    EXPECT_EQ(cams[0].label, "Rear");
    EXPECT_EQ(cams[1].label, "Mast");
    EXPECT_EQ(camera_config::topicFor(cams[0]),
              QStringLiteral("/logitech/image_raw_decoded"));
}

TEST(CameraConfig, RejectsEntryMissingFields) {
    QString err;
    auto cams = camera_config::parse(R"({"cameras":[{"label":"x"}]})", &err);
    EXPECT_TRUE(cams.empty());
    EXPECT_FALSE(err.isEmpty());
}

TEST(CameraConfig, RejectsMalformedJson) {
    QString err;
    auto cams = camera_config::parse("{nope", &err);
    EXPECT_TRUE(cams.empty());
    EXPECT_FALSE(err.isEmpty());
}

TEST(CameraConfig, CatPlaceholdersResolve) {
    auto cat0 = camera_config::catFor(0);
    ASSERT_FALSE(cat0.isEmpty());
    EXPECT_TRUE(QFile::exists(cat0));
    EXPECT_FALSE(cat0.contains(QStringLiteral("cat_square")));
    EXPECT_TRUE(camera_config::catFor(100).isEmpty() == false);  // wraps, never empty
    EXPECT_FALSE(camera_config::catError().isEmpty());
}

TEST(CameraConfig, RejectsEmptyCameraList) {
    QString err;
    auto cams = camera_config::parse(R"({"cameras":[]})", &err);
    EXPECT_TRUE(cams.empty());
    EXPECT_FALSE(err.isEmpty());
}
