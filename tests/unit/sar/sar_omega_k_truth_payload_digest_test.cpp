#include <gtest/gtest.h>

#include <string>

#include "sar/imaging/SarOmegaKTruthPayloadDigest.h"

namespace sar {
namespace imaging {
namespace {

OmegaKTruthDigestRequest RequestFor(const std::string& payload,
                                    const std::string& digest) {
  OmegaKTruthDigestRequest request;
  request.request_id = 101U;
  request.payload_bytes.assign(payload.begin(), payload.end());
  request.declared_sha256 = digest;
  return request;
}

TEST(SarOmegaKTruthPayloadDigestTest, MatchesKnownSha256Vectors) {
  const OmegaKTruthDigestResult empty = VerifyOmegaKTruthPayloadDigest(RequestFor(
      "", "e3b0c44298fc1c149afbf4c8996fb924"
          "27ae41e4649b934ca495991b7852b855"));
  EXPECT_EQ(empty.status, OmegaKTruthDigestStatus::kMatched);

  const OmegaKTruthDigestResult abc = VerifyOmegaKTruthPayloadDigest(RequestFor(
      "abc", "BA7816BF8F01CFEA414140DE5DAE2223"
             "B00361A396177A9CB410FF61F20015AD"));
  EXPECT_EQ(abc.status, OmegaKTruthDigestStatus::kMatched);
  EXPECT_EQ(abc.computed_sha256,
            "ba7816bf8f01cfea414140de5dae2223"
            "b00361a396177a9cb410ff61f20015ad");
}

TEST(SarOmegaKTruthPayloadDigestTest, ReportsTamperedPayloadAsMismatch) {
  const OmegaKTruthDigestResult result = VerifyOmegaKTruthPayloadDigest(RequestFor(
      "abd", "ba7816bf8f01cfea414140de5dae2223"
             "b00361a396177a9cb410ff61f20015ad"));
  EXPECT_EQ(result.status, OmegaKTruthDigestStatus::kMismatched);
  EXPECT_FALSE(result.computed_sha256.empty());
}

TEST(SarOmegaKTruthPayloadDigestTest, RejectsInvalidRequestAtomically) {
  OmegaKTruthDigestRequest request = RequestFor("abc", "invalid");
  const OmegaKTruthDigestResult result = VerifyOmegaKTruthPayloadDigest(request);
  EXPECT_EQ(result.reason, OmegaKTruthDigestReason::kInvalidDeclaredDigest);
  EXPECT_TRUE(result.computed_sha256.empty());

  request = RequestFor("abc",
                       "ba7816bf8f01cfea414140de5dae2223"
                       "b00361a396177a9cb410ff61f20015ad");
  request.request_id = 0U;
  EXPECT_EQ(VerifyOmegaKTruthPayloadDigest(request).reason,
            OmegaKTruthDigestReason::kInvalidRequestId);
}

}  // namespace
}  // namespace imaging
}  // namespace sar
