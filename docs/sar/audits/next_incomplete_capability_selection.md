# Next Incomplete SAR Capability Selection

> **⚠️ 已被 superseded(2026-06-24)**:本文档将 PGA 列为"下一个待开发能力"。经
> `pga_autofocus_closure_phase_a_verdict.md` 阶段 A 证据判定,PGA 闭环**不实现**
> (MoCo 已完全修复直线场景散焦,所有扰动档位 NRMS<0.17)。本文档保留作历史选型记录。

## Selection

The next independent incomplete SAR capability is Phase Gradient Autofocus
(PGA), beginning with a deterministic support-selection and phase-gradient
truth contract.

## Evidence

The existing autofocus reference work provides controlled residual phase-error
injection and reference diagnostics, but explicitly defers:

- PGA support-region selection;
- phase-gradient estimation truth;
- phase unwrap;
- iterative stopping criteria;
- production complex-image correction and session integration.

The Omega-K repository-side physical acceptance path is now ready and blocked
only by external physical truth, so extending its internal acceptance wrappers
would not reduce the actual blocker.

## Next Boundary

The next stage shall freeze deterministic PGA support selection and
phase-gradient truth. It shall not yet authorize phase integration, unwrap,
iterative correction, entropy optimization, or production image modification.
