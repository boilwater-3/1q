#include "sar/imaging/SarOmegaKTruthIngestion.h"

#include "sar/imaging/SarOmegaKTruthPayloadDigest.h"

namespace sar {
namespace imaging {

namespace {

OmegaKTruthIngestionResult Reject(const OmegaKTruthIngestionRequest& request,
                                  OmegaKTruthIngestionReason reason) {
  OmegaKTruthIngestionResult result;
  result.request_id = request.request_id;
  result.reason = reason;
  return result;
}

}  // namespace

OmegaKTruthIngestionResult IngestOmegaKTruth(
    const OmegaKTruthIngestionRequest& request) {
  if (request.request_id == 0U) {
    return Reject(request, OmegaKTruthIngestionReason::kInvalidRequestId);
  }
  const OmegaKTruthManifestParseResult parsed =
      ParseOmegaKTruthManifest(request.manifest_text);
  if (parsed.status != OmegaKTruthManifestStatus::kParsed) {
    return Reject(request, OmegaKTruthIngestionReason::kManifestRejected);
  }
  OmegaKTruthDigestRequest digest_request;
  digest_request.request_id = request.request_id;
  digest_request.payload_bytes = request.payload_bytes;
  digest_request.declared_sha256 = parsed.manifest.digest_sha256;
  const OmegaKTruthDigestResult digest = VerifyOmegaKTruthPayloadDigest(digest_request);
  if (digest.status == OmegaKTruthDigestStatus::kRejected) {
    return Reject(request, OmegaKTruthIngestionReason::kDigestRejected);
  }
  if (digest.status != OmegaKTruthDigestStatus::kMatched) {
    return Reject(request, OmegaKTruthIngestionReason::kDigestMismatch);
  }

  OmegaKTruthIngestionResult result;
  result.request_id = request.request_id;
  result.status = OmegaKTruthIngestionStatus::kSucceeded;
  result.reason = OmegaKTruthIngestionReason::kNone;
  result.manifest = parsed.manifest;
  result.payload_bytes = request.payload_bytes;
  result.computed_sha256 = digest.computed_sha256;
  return result;
}

}  // namespace imaging
}  // namespace sar
