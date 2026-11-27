#include "../_imageReader/imageReaderFromFile.h"

#include <gtest/gtest.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <unistd.h>

#include <cstdio>
#include <filesystem>
#include <memory>
#include <set>

// ============================================================================
// FIXTURE
// ============================================================================

class ImageReaderFromFileTest : public ::testing::Test {
   protected:
    ImageReaderFromFile reader;
    std::string tempDir;

    void SetUp() override {
        // Use a temp directory unique per test (and per process).
        const ::testing::TestInfo* info = ::testing::UnitTest::GetInstance()->current_test_info();
        tempDir = std::filesystem::temp_directory_path().string() + "/imageReaderFileTest_" + info->test_suite_name() +
                  "_" + info->name() + "_" + std::to_string(::getpid());
        std::filesystem::remove_all(tempDir);  // clear any stale leftover
        std::filesystem::create_directories(tempDir);
    }

    void TearDown() override { std::filesystem::remove_all(tempDir); }

    // Create a BGR image: black background with white (255,255,255) pixels at given locations
    std::string createTestImage(int width, int height, const std::vector<Eigen::Vector2i>& whitePixels) {
        cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
        for (const auto& p : whitePixels) {
            // OpenCV: mat.at<Vec3b>(row, col) = mat.at<Vec3b>(y, x)
            img.at<cv::Vec3b>(p[1], p[0]) = cv::Vec3b(255, 255, 255);
        }
        std::string path = tempDir + "/test_image.png";
        cv::imwrite(path, img);
        return path;
    }

    // Create a BGR image with specific gray values at given pixel locations
    std::string createGrayValueImage(int width,
                                     int height,
                                     const std::vector<std::pair<Eigen::Vector2i, uint8_t>>& pixels) {
        cv::Mat img = cv::Mat::zeros(height, width, CV_8UC3);
        for (const auto& [p, val] : pixels) {
            img.at<cv::Vec3b>(p[1], p[0]) = cv::Vec3b(val, val, val);
        }
        std::string path = tempDir + "/gray_test_image.png";
        cv::imwrite(path, img);
        return path;
    }

    // Collect the (x, y) coordinates of all non-zero pixels of a thresholded image
    std::set<std::pair<int, int>> collectNonZeroPixels(const cv::Mat& image) {
        std::vector<cv::Point> locations;
        cv::findNonZero(image, locations);
        std::set<std::pair<int, int>> result;
        for (const auto& [x, y] : locations) {
            result.insert({x, y});
        }
        return result;
    }
};

// ============================================================================
// SETTER / GETTER TESTS
// ============================================================================

TEST_F(ImageReaderFromFileTest, SettersGetters) {
    reader.setBlurSize(5);
    EXPECT_EQ(5, reader.getBlurSize());

    reader.setPixelThreshold(128.0F);
    EXPECT_FLOAT_EQ(128.0F, reader.getPixelThreshold());

    reader.setFileName("/some/path.png");
    EXPECT_EQ("/some/path.png", reader.getFileName());

    reader.setSaveImages(true);
    EXPECT_TRUE(reader.getSaveImages());

    reader.setSaveDir("/tmp/save");
    EXPECT_EQ("/tmp/save", reader.getSaveDir());
}

// ============================================================================
// ERROR HANDLING
// ============================================================================

TEST_F(ImageReaderFromFileTest, ThrowsOnEmptyFilename) { EXPECT_THROW(reader.getImage(), std::invalid_argument); }

// ============================================================================
// IMAGE SIZE
// ============================================================================

TEST_F(ImageReaderFromFileTest, GetFullImageSize) {
    std::string path = createTestImage(30, 20, {});
    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    // getImage reads the file. readImageFromFile sets the full image size.
    reader.getImage();

    Eigen::Vector2i size = reader.getFullImageSize(0);
    EXPECT_EQ(30, size[0]);
    EXPECT_EQ(20, size[1]);
}

// ============================================================================
// CHECK FOR NEW IMAGE
// ============================================================================

TEST_F(ImageReaderFromFileTest, CheckForNewImageAlwaysReturnsOne) {
    EXPECT_EQ(1, reader.getCurrentImageTimeTag(0, 0));
    EXPECT_EQ(1, reader.getCurrentImageTimeTag(0, 999));
    EXPECT_EQ(1, reader.getCurrentImageTimeTag(42, 12345));
}

// ============================================================================
// NON-ZERO PIXEL LOCATIONS
// ============================================================================

TEST_F(ImageReaderFromFileTest, NonZeroPixelLocations) {
    // Create 20x20 black image with 3 known white pixels
    std::vector<Eigen::Vector2i> whitePixels = {
        Eigen::Vector2i(5, 5), Eigen::Vector2i(10, 10), Eigen::Vector2i(15, 15)};
    std::string path = createTestImage(20, 20, whitePixels);

    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    auto found = collectNonZeroPixels(reader.getImage());

    EXPECT_EQ(3u, found.size());
    EXPECT_TRUE(found.count({5, 5}));
    EXPECT_TRUE(found.count({10, 10}));
    EXPECT_TRUE(found.count({15, 15}));
}

// ============================================================================
// THRESHOLDED IMAGE
// ============================================================================

TEST_F(ImageReaderFromFileTest, GetImageReturnsThresholdedImage) {
    std::vector<Eigen::Vector2i> whitePixels = {Eigen::Vector2i(5, 5), Eigen::Vector2i(15, 3)};
    std::string path = createTestImage(20, 10, whitePixels);

    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    const cv::Mat image = reader.getImage();

    ASSERT_EQ(CV_8UC1, image.type());
    EXPECT_EQ(10, image.rows);
    EXPECT_EQ(20, image.cols);
    EXPECT_EQ(2, cv::countNonZero(image));
    EXPECT_EQ(255, image.at<uint8_t>(5, 5));
    EXPECT_EQ(255, image.at<uint8_t>(3, 15));
}

// ============================================================================
// THRESHOLD FILTERING
// ============================================================================

TEST_F(ImageReaderFromFileTest, ThresholdFiltering) {
    // Pixels with gray value 100 and 200
    std::string path = createGrayValueImage(20, 20, {{Eigen::Vector2i(5, 5), 100}, {Eigen::Vector2i(15, 15), 200}});

    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(150);

    auto found = collectNonZeroPixels(reader.getImage());

    // Only the pixel with value 200 should pass threshold 150
    EXPECT_TRUE(found.count({15, 15}));
    EXPECT_FALSE(found.count({5, 5}));
}

// ============================================================================
// BLUR EFFECT
// ============================================================================

TEST_F(ImageReaderFromFileTest, BlurEffect) {
    // Single bright pixel — with blur=1 (no blur), only that pixel should appear
    std::string path = createTestImage(20, 20, {Eigen::Vector2i(10, 10)});

    // blur=1: no blurring, should find exactly 1 pixel
    reader.setFileName(path);
    reader.setBlurSize(1);
    reader.setPixelThreshold(50);

    auto foundNoBlur = collectNonZeroPixels(reader.getImage());

    // blur=5: blurring spreads the pixel, more neighbors pass threshold
    reader.setBlurSize(5);
    auto foundWithBlur = collectNonZeroPixels(reader.getImage());

    EXPECT_EQ(1u, foundNoBlur.size());
    // With blur, more pixels should pass (or fewer if blur dilutes below threshold)
    // The key test: the results should differ
    EXPECT_NE(foundNoBlur.size(), foundWithBlur.size());
}
