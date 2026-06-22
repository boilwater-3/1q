# Omega-K Truth Payload Digest Verification Contract

## Purpose

This contract separates truth-manifest digest syntax validation from
cryptographic payload integrity verification.

## Request

The verifier request shall provide:

- a nonzero request identifier;
- the exact payload bytes as acquired;
- the manifest-declared SHA-256 digest.

The verifier shall not parse, normalize, re-encode, or otherwise modify the
payload before hashing.

## Result

The result shall report:

- the request identifier;
- the computed lowercase SHA-256 digest;
- whether the computed digest exactly matches the declared digest,
  case-insensitively;
- an explicit rejection reason for invalid identifiers or digest syntax.

## Atomic Behavior

Invalid requests shall not publish a computed digest or a partial match result.

## Boundary

A matching digest establishes payload integrity relative to the manifest. It
does not establish source independence, physical correctness, or image
acceptance.
