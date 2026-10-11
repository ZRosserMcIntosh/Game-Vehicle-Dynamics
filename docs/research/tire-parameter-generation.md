# Tire parameter generation without proprietary data

Source: [`tire_generator.hpp`](../../src/core/include/ord/tire_generator.hpp) / [`.cpp`](../../src/core/src/tire_generator.cpp).

## Why

Full Magic Formula coefficient files (`.tir`) are produced by tire makers and test labs, and are usually licensed. Some public example files have unclear provenance; [alternatives.md](alternatives.md) notes a Chrono `.tir` that was scaled from an ADAMS/Car example. We need dozens of plausible tires for the game's car roster, and we must be free to redistribute them.

## Approach

From a tire's public description (size string and category), derive the reduced MF parameter set used by the core.

| Parameter | Mapping | Typical range targeted |
|---|---|---|
| Loaded radius | rim/2 + width × aspect, minus static deflection | — |
| Vertical stiffness | 250 kN/m × (width/225)^0.7 × (55/aspect)^0.5 | 180–350 kN/m |
| Peak μ (dry) | by category: Eco 0.95 … Slick 1.55 | road 0.9–1.2, slick 1.4–1.7 |
| Load sensitivity | −0.07 … −0.15 per unit dfz (slicks more sensitive) | — |
| Cornering stiffness | 16 Fz0/rad × (width/225)^0.4 × (55/aspect)^0.35 × category factor | 12–25 Fz0/rad |
| Lateral relaxation length | 1.8 × section width × √(aspect/55) | 0.3–0.6 m |
| Patch half-length | √(2 r δ) | 0.05–0.12 m |
| Rolling resistance | by category 0.008–0.015 | — |

The ranges are typical textbook values, e.g. Pacejka (2012) ch. 3–4 for MF parameter magnitudes and relaxation lengths, and Milliken & Milliken (1995) for passenger and racing tire μ and stiffness. They are deliberately round. They are *not* fitted to, and do not describe, any manufacturer's product.

## Status and next steps

- Unit test: parsing, monotonic trends (wider → stiffer, slick → more grip, eco → less rolling resistance), and acceptance by the MF and the vehicle validator.
- Next: calibrate against an openly licensed measured dataset. The CC BY 4.0 tire bench data noted in [alternatives.md §9](alternatives.md) is a candidate; check its licence and provenance before use. Store each fit with its source in `docs/research/`.
- Tires generated this way are a credible *starting point* for vehicle authors. Handling sign-off still comes from benchmarks and driving.
