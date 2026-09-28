# VISION-LIVESTITCH, Explained From Zero

> Written for 3am brain. No prior assumed beyond "I know what a photo is." If a section still doesn't click, that's a doc bug, not a you problem — come back and say which part.

This doc has five parts, build on each other in order:

1. [The mental model](#1-the-mental-model-what-is-image-stitching-actually) — what stitching *is*, with no code at all.
2. [How this project turns that idea into a running system](#2-how-this-project-turns-that-idea-into-a-running-system)
3. [The technical pieces, one at a time, each with an analogy first](#3-the-technical-pieces-one-at-a-time)
4. [Your actual bug, explained with the same analogies](#4-your-actual-bug-explained-with-the-same-analogies)
5. [The fixes, and why each one helps](#5-the-fixes-and-why-each-one-helps)

---

## 1. The Mental Model: What Is Image Stitching, Actually?

Forget drones and code for a second. Imagine you're building a giant photo of your street by taking 20 overlapping photos while walking down the sidewalk, then laying them out on a table like a collage so they line up into one continuous picture.

To make photo #2 line up with photo #1, your brain does something like this without you noticing:

1. **Spot landmarks that appear in both photos** — a mailbox, a specific tree, a crack in the pavement. These are things you can recognize again even though the camera moved a bit.
2. **Match up "this mailbox in photo 1" with "this same mailbox in photo 2."**
3. **Figure out how photo 2 needs to shift, rotate, and maybe stretch** so that the mailbox in photo 2 lands exactly on top of the mailbox in photo 1.
4. **Slide photo 2 into place** using that shift/rotate/stretch, and glue it down.
5. **Blend the seam** so you can't tell where photo 1 ends and photo 2 begins.

That's it. That's the entire idea. Everything in this codebase is a computer's version of steps 1–5, done automatically, frame after frame, for a video coming off a drone instead of a person's photo album.

**Why is a drone video harder than a photo album?** Two reasons this project cares about a lot:

- **Speed.** A drone doesn't wait for you to finish stitching photo 5 before it sends photo 6. The system has to keep up in *near real-time*, so it can't afford to be clever and slow — it has to be simple and fast.
- **No time to double-check everything.** A human doing this by hand would notice immediately if photo 12 landed in a weird spot and would nudge it back. The automated pipeline doesn't have that instinct built in unless someone (you) puts a check in place for it. That's foreshadowing for §4.

---

## 2. How This Project Turns That Idea Into a Running System

Think of it as an assembly line with four stations. A picture moves down the line, station by station:

```
[Drone camera] → [Sender] → [Receiver] → [Watchdog/Trigger] → [Mapping Engine (stitcher)] → [Result on screen]
```

1. **Drone camera + Sender**: takes a photo, turns it into a JPEG, sends it over the network to the ground station. (See [SOCKET.md](./SOCKET.md) for the wire-level detail — for now just picture "photo goes over the network.")
2. **Receiver**: sits at the ground station, catches incoming photos, saves each one to a folder on disk. Each drone gets its own folder ("session") so multiple drones don't mix their photos together — see [MULTI_UAV_ARCHITECTURE.md](./MULTI_UAV_ARCHITECTURE.md).
3. **Watchdog/Trigger**: a program that's *watching* that folder like a mailbox. The moment enough new photos pile up (default: every 5), it says "okay, time to stitch these."
4. **Mapping Engine**: this is the actual "steps 1–5" from §1 above, done in code. This is `src/Combiner.py`, and it's the star of this whole document.

The output is a growing image (the "mosaic" or "orthomosaic") that you can watch update on a dashboard, live, as the drone flies.

---

## 3. The Technical Pieces, One at a Time

Each piece below maps to one step from the mental model in §1. Analogy first, then what it's actually called in the code.

### 3.1 "Spot the landmarks" → Keypoint Detection (SIFT)

**Analogy**: Imagine putting a small sticky flag on every corner, edge, or distinctive blob in a photo — anywhere your eye would say "yeah, I'd recognize that spot again." A blank sky gets no flags (nothing distinctive). A cluttered rooftop gets lots of flags.

**Technical**: SIFT (Scale-Invariant Feature Transform) finds points where local image gradients change sharply in more than one direction (corners, blobs) and are stable across changes in scale and rotation. For each such point it builds a 128-dimensional **descriptor** vector from the histogram of gradient orientations in its neighborhood — this vector is the "fingerprint" that survives the drone's altitude and heading changing slightly frame to frame.

**In code**: `cv2.SIFT_create(1800)` — finds up to 1800 of these "flaggable" spots (called **keypoints**) per image, and for each one, computes a little numeric fingerprint (called a **descriptor**) that describes what that spot's local texture looks like.

### 3.2 "Match up the mailbox" → Feature Matching

**Analogy**: You've got a pile of flagged spots from photo 1 and a pile from photo 2. Now you go through and pair them up: "this flag in photo 1 and this flag in photo 2 — same landmark."

**The tricky part**: what if two different spots have *similar-looking* fingerprints? You could pair up the wrong ones. To guard against that, the code uses a trick called the **ratio test**: for each flag in photo 2, it finds the *two* best-looking candidate matches in photo 1. If the best one is *clearly* better than the second-best one, we trust it ("this landmark is distinctive enough that I'm not confusing it with a lookalike"). If the best and second-best are close, we throw the match away — too risky, might be a mix-up.

**Technical**: `BFMatcher` compares descriptor vectors by Euclidean distance — smaller distance means more similar fingerprints. For each descriptor in image 2, `knnMatch(k=2)` returns its single nearest neighbor (`m`) and second-nearest neighbor (`n`) in image 1's descriptor set. Lowe's ratio test accepts the match only if `m.distance < ratio * n.distance` — i.e., the best candidate must beat the runner-up by a comfortable margin, not just barely win. This project uses `0.55` instead of Lowe's commonly cited `0.75`, trading fewer total matches for a lower false-positive rate.

**In code**: `BFMatcher().knnMatch(..., k=2)` finds those two best candidates; the line `m.distance < 0.55 * n.distance` is the "clearly better" check. (`0.55` is a strictness dial — lower means "only trust very confident matches.")

**Why this matters for your bug**: this safety check has a blind spot when *many* things in the scene genuinely look alike. Hold that thought for §4.

### 3.3 "Figure out the shift/rotate/stretch" → Transform Estimation

**Analogy**: Once you know "this landmark in photo 2 corresponds to this landmark in photo 1" for a handful of pairs, you can do some geometry to work out exactly how photo 2 needs to move (slide left/right/up/down, rotate a bit, maybe resize) so all those landmark pairs line up simultaneously.

**Technical**: an affine transform has 4 degrees of freedom (translation x/y, rotation, uniform scale) and needs a minimum of ~3 point correspondences to solve; a full homography has 8 degrees of freedom (adds perspective/skew) and needs a minimum of 4. `cv2.estimateAffinePartial2D` and `cv2.findHomography(..., method=cv2.RANSAC)` both run RANSAC internally: they repeatedly sample a small random subset of the matched points, fit a candidate transform, count how many of the *other* matches agree with it within a tolerance ("inliers"), and keep the candidate with the most agreement. The final transform is then refit using all inliers.

**In code**: this "how to move it" answer is a matrix (a grid of numbers) called a **homography** (or a simpler version called an **affine transform** when there's no perspective distortion — just slide/rotate/scale). The code tries the simpler, faster affine first (`cv2.estimateAffinePartial2D`), and only falls back to the more flexible (but more failure-prone) full homography (`cv2.findHomography`) if the simple one doesn't work.

You need at least a few matched landmark pairs to solve for this matrix — the code requires **at least 4**.

### 3.4 "Slide it into place, frame after frame" → Chained Matrix Multiplication

**Analogy**: This is the part that's a little unusual and worth slowing down on. Imagine you're blindfolded, walking down a hallway, and someone tells you "take 3 steps forward, then 1 step right" — you do that, then they tell you the *next* instruction relative to where you now think you are. You never get to look around and confirm you're actually where you think you are. You just trust the last instruction, add the new one on top, and keep walking.

That's exactly what this pipeline does with images. It never re-checks a frame against a "ground truth" map. It just says: "frame 47 was placed at position X. Frame 48's transform says move a little from there. So frame 48 goes at X + a little." And frame 49 builds on top of *that* answer, and so on.

**Why do it this way instead of the "smarter" way?** The smarter way (used by most panorama software) is called **bundle adjustment** — it looks at *all* the frames together and adjusts them all at once to minimize total error, like solving a big jigsaw puzzle all at once instead of one piece at a time. That's more accurate, but much slower — too slow for real-time video. This project deliberately trades some accuracy for speed by chaining one-step-at-a-time instead. That's a documented, intentional design choice — see [TECHNICAL.md §1](./TECHNICAL.md#1-project-overview).

**The catch**: if the blindfolded walker mishears one instruction (a bad transform), every step after it inherits that mistake, because nobody ever double-checks against reality. That's the root of both the *slow drift* problem (§10.1 in TECHNICAL.md — errors pile up gradually over 50-100 frames) and your *sudden* problem (§4 below — one specific instruction gets misheard badly).

**Technical**: `H_rel` is the estimated transform mapping frame N into frame N-1's coordinate space. `H_global` maps any frame directly into frame 0's (the mosaic's) coordinate space. Because matrix multiplication composes transforms, `H_global_current = H_global_prev × H_rel` gives frame N's position in the mosaic without ever re-deriving it from scratch. The matrix is renormalized (`H / H[2,2]`) after each multiply to keep the homogeneous-coordinate scale stable. Critically, each `H_rel` carries whatever estimation error came out of §3.3 — and because these errors multiply together rather than average out, they compound roughly geometrically over the sequence rather than canceling.

**In code**: `H_global_current = H_global_prev @ H_rel` — "where I am now" = "where I was" combined with "how I just moved."

### 3.5 "Only look where you expect the overlap" → Adaptive ROI Prediction

**Analogy**: If you just walked 3 steps forward, you don't need to scan the *entire* hallway to find the wall you're now near — you can guess roughly where it should be and only look there. Saves time.

**Technical**: the code inverts `H_rel_prev` and projects the new image's four corners through it, which estimates where the previous frame's footprint falls inside the new frame's coordinate space. The bounding box of those projected corners, padded 15%, becomes the crop that SIFT runs on. Keypoints found inside the crop are then offset by `(x_start, y_start)` to translate their coordinates back into full-image space before matching, so downstream code never needs to know a crop happened. If the predicted box is smaller than 100×100 px, the code gives up and uses the full frame instead.

**In code**: the system uses the *previous* frame's transform to predict where the new frame's overlap with the old mosaic should be, and only runs the expensive keypoint-detection step (§3.1) inside that predicted region, with some safety margin (15% padding) — instead of scanning the whole photo. This is a speed optimization. It's not a correctness check — it doesn't verify anything, it just narrows *where to look*.

### 3.6 "Blend the seam" → Feather Blending

**Analogy**: When two photos overlap, you don't want a hard visible line where one ends and the other begins. So near the edge of the overlap, you gradually fade from "mostly photo 1" to "mostly photo 2."

**Technical**: for each pixel inside the overlap's bounding box, `cv2.distanceTransform` computes its distance to the nearest non-overlap edge of each image's mask separately (`d1`, `d2`). The blend weight is `alpha = (d1 / (d1 + d2 + eps))^3` — a pixel deep inside photo 1's territory gets `alpha` near 1 (mostly photo 1), a pixel near photo 1's edge but deep in photo 2's territory gets `alpha` near 0. Cubing the ratio steepens the falloff so the transition band is narrower and doesn't look "muddy." Restricting all of this to the overlap's bounding box (rather than the full canvas) is what keeps the cost at O(overlap area) instead of O(canvas area).

**In code**: `ROIfeatherBlender` — computes, for every pixel in the overlap zone, "how close is this pixel to the edge of photo 1's coverage vs. photo 2's coverage," and blends proportionally. Cheap because it only does this math inside the small overlapping rectangle, not the whole canvas.

---

## 4. Your Actual Bug, Explained With the Same Analogies

Now the payoff — let's re-tell what happened to your colleague's test flight using the analogies above, no new jargon.

**The scene**: a farmland corridor — rows of crops, repeating over and over, visually almost identical to each other. Picture a hallway with **a hundred identical doors in a row**, each one indistinguishable from the next.

**What went wrong**: the blindfolded walker from §3.4 reached out, felt a doorknob, and matched it to "yep, that's door #7, same one as last time" — except it was actually door #8. Because all the doors look the same, this was a *confident*, clean-feeling match (the ratio test in §3.2 was satisfied — there was nothing else nearby that looked *more* like door #7, so the "am I sure?" check passed) — it just happened to be confidently wrong.

When enough of these confidently-wrong door-matches line up with each other (they all agree "we moved zero doors" or "we moved exactly one door," because they're all fooled the same way), the transform-estimation step (§3.3) computes a transform that satisfies all of them — geometrically consistent, but describing the wrong motion. In this case, it landed close to "we didn't move at all," so the new photo got glued down almost exactly on top of the previous one — ghosting.

**Technical**: this is SIFT descriptor aliasing on periodic texture. Crop rows repeat at a near-constant spatial period, so a descriptor computed at row *k* is nearly identical to one computed at row *k±1, k±2, ...*. The ratio test (§3.2) only rejects a match when the best and second-best *candidates within that single knnMatch call* are close together — it has no way to know that "row 7" and "row 8" are two *different* keypoints elsewhere in the descriptor space that happen to describe near-identical texture. If the true corresponding row wins clearly against everything else nearby, the ratio test passes confidently, even though "everything else nearby" didn't include the actual right answer at all. Once ≥4 such matches exist, RANSAC inside `estimateAffinePartial2D` (§3.3) fits a transform that's internally self-consistent (the aliased points all agree with each other, since they're all off by the same row-period) and gets accepted, because — per §2 of [DRIFT_MISREGISTRATION.md](./DRIFT_MISREGISTRATION.md) — `Combiner.py` has exactly one quality gate (`len(matches) >= 4`, line 258) and zero geometric-plausibility gates.

**Why "random," not every N frames?** This isn't a steady bias — it's a coincidence. It only happens when the crop row spacing in that particular field, combined with how far the drone actually moved in that particular frame gap, happen to line up in a way that fools the matcher. Different field, different drone speed, different result. That's why it's unpredictable rather than following a fixed pattern.

**Why doesn't the "only look nearby" trick (§3.5) save us?** Because narrowing *where* to look doesn't help when the thing you're confused about is *"which repeated pattern is this,"* not *"where roughly is the overlap."* If anything, narrowing the search area can crop out a unique landmark near the image edge (like a road, which doesn't repeat) that would've broken the tie.

**The part that should bother you most**: nowhere in this pipeline does anything ever ask "wait, does this transform even make sense?" Once four-or-more door-matches agree with each other, the code accepts the resulting transform *unconditionally* — no second opinion, no plausibility check. That's the actual bug. Not "SIFT is bad" or "ROI is broken" — it's that **there is no verification step at all.**

---

## 5. The Fixes, and Why Each One Helps

Full trade-off table lives in [DRIFT_MISREGISTRATION.md §6](./DRIFT_MISREGISTRATION.md#6-candidate-solutions--trade-offs) — here's the plain-language version of the two recommended ones.

### Fix A: Get a second opinion from GPS ("does this feel right?")

The drone already knows roughly how far it moved between two photos — that's what GPS is for. Your colleague's code is even already converting GPS coordinates into "meters moved" (`data_matrix[i, 0]` / `[i, 1]`) — it's just never actually *used* for anything past that point.

**The fix**: after estimating a transform from the (possibly door-confused) landmark matches, compare "how far did the transform say we moved" against "how far did GPS say we moved." If those two numbers disagree by a lot, don't trust the transform — reject it and fall back to something safer (like re-trying on the full photo instead of just the predicted overlap zone).

This is like the blindfolded walker also wearing a pedometer. Even blindfolded and door-confused, if the pedometer says "you took 3 steps" and your hand-feel says "we didn't move at all," you know something's wrong — even without being able to see.

**Technical**: `dataMatrix[i, 0:2]` already holds each frame's local `(x, y)` in meters, converted from GPS lat/lon (`x = (lon - origin_lon) * 111320 * cos(origin_lat)`, `y = (lat - origin_lat) * 110540`). The fix is to compute `expected_translation = dataMatrix[index, 0:2] - dataMatrix[index-1, 0:2]` (converted into the same pixel/scale units the homography operates in), extract the translation component from the candidate `H_rel_3x3` (its `[0:2, 2]` entries for an affine, or the mapped origin for a full homography), and compare the two vectors' magnitude and direction. If they diverge past a tolerance (needs tuning against typical consumer-GNSS error, ~1-3m), reject `H_rel` and take a fallback path instead of chaining it into `H_global`. This is currently unimplemented — see [DRIFT_MISREGISTRATION.md §6, item 1](./DRIFT_MISREGISTRATION.md#6-candidate-solutions--trade-offs).

### Fix B: Check how many landmarks actually agreed ("was this a landslide or a squeaker?")

When the code computes a transform, the underlying algorithm (RANSAC) actually keeps track of *which* matched pairs it used to reach its answer and which ones it threw out as outliers — but the code currently throws that information away and only checks "were there at least 4 matches," not "did most of them actually agree."

**The fix**: check that a healthy majority of matches agree with the final transform (a high "inlier ratio"), not just a bare minimum count. This won't catch the door-confusion case on its own (all the fooled matches *do* agree with each other, that's the whole problem) — but it's a free, cheap safety net for a different, more common failure (too few or too scattered matches), and costs almost nothing to add since the information already exists, it's just being discarded today.

**Technical**: `cv2.estimateAffinePartial2D(src_pts, dst_pts)` returns `(A, inliers)`, but `Combiner.py:100` currently does `A, _ = cv2.estimateAffinePartial2D(...)`, discarding `inliers` — a mask array the same length as the match list, marking which matches RANSAC judged consistent with the fitted model. The fix: keep that mask, compute `inlier_ratio = inliers.sum() / len(matches)`, and reject the transform if the ratio falls below some threshold (e.g. 0.6-0.7), in addition to the existing raw-count check at line 258. Note the ceiling on this fix: RANSAC's inlier test only measures *self-consistency* among the given matches, not agreement with ground truth — so a fully-aliased match set (all matches confidently wrong in the same direction) will still show a high inlier ratio. That's exactly why this is paired with the GPS check (Fix A) rather than used alone.

### Why not just always use the fancy global method (bundle adjustment)?

Because that's the "look at the whole jigsaw puzzle at once and adjust everything" approach — much more accurate, but too slow to run in real time on video. This project chose speed on purpose. The fixes above are about adding a *cheap sanity check* to the fast method, not replacing the fast method with the slow one.

---

## Quick Glossary (for skimming back later)

| Term | Plain meaning |
|---|---|
| **Keypoint** | A "flagged," distinctive spot in an image, worth remembering. |
| **Descriptor** | The numeric fingerprint describing what that spot looks like. |
| **SIFT** | The specific algorithm used to find keypoints + descriptors. |
| **Matching** | Pairing up "this spot in photo A" with "this spot in photo B." |
| **Ratio test** | The "am I sure this match isn't a lookalike mix-up?" check. |
| **Affine transform** | A simple move: slide + rotate + uniform resize, no perspective warp. |
| **Homography** | A more flexible move that also handles perspective distortion. |
| **H_rel** | "How much did we move, relative to the last frame." |
| **H_global** | "Where are we now, relative to the very first frame" (built by chaining `H_rel`s together). |
| **Chained matrix multiplication** | The "blindfolded walker" approach — build position from a sequence of relative steps, never double-checked against ground truth. |
| **Bundle adjustment** | The "look at the whole puzzle at once" alternative — more accurate, much slower. Not used here on purpose. |
| **ROI (Region of Interest)** | The predicted "probably here" zone the code searches, instead of the whole photo, to save time. |
| **Drift** | Slow, gradual misalignment building up over many frames — different bug from the one in this doc. |
| **Feather blending** | Fading the seam between two overlapping photos so it's invisible. |
| **Inlier / inlier ratio** | Of the matched landmark pairs, how many actually agree with the final computed transform. |

---

*Next read: [DRIFT_MISREGISTRATION.md](./DRIFT_MISREGISTRATION.md) for the full technical write-up with exact line numbers and the action checklist. [TECHNICAL.md](./TECHNICAL.md) for the complete module-by-module reference.*
