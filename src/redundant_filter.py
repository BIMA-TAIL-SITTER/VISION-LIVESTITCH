import cv2
import numpy as np


# ── tuneable thresholds ────────────────────────────────────────────────────────
HASH_HAMMING_THRESHOLD   = 8     # 0‒64; lower = stricter identical check
PIXEL_DIFF_THRESHOLD     = 0.02  # fraction of pixels that changed
FEATURE_MATCH_THRESHOLD  = 0.85  # ratio of matched features; higher = more similar


def _phash(image: np.ndarray, hash_size: int = 8) -> np.ndarray:
    """Perceptual hash: resize → grayscale → DCT → median threshold."""
    gray    = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    small   = cv2.resize(gray, (hash_size * 4, hash_size * 4))
    # DCT on float
    dct     = cv2.dct(np.float32(small))
    dct_low = dct[:hash_size, :hash_size]          # top-left = low frequencies
    median  = np.median(dct_low)
    return (dct_low > median).flatten()             # 64-bit boolean array


def _hamming(h1: np.ndarray, h2: np.ndarray) -> int:
    return int(np.count_nonzero(h1 != h2))


def _pixel_diff_ratio(img_a: np.ndarray, img_b: np.ndarray) -> float:
    """
    Downsample both to 64x64, compute per-pixel absolute difference.
    Returns fraction of pixels that changed significantly (>10 intensity units).
    """
    SIZE   = (64, 64)
    small_a = cv2.resize(img_a, SIZE)
    small_b = cv2.resize(img_b, SIZE)
    diff    = cv2.absdiff(small_a, small_b)
    gray    = cv2.cvtColor(diff, cv2.COLOR_BGR2GRAY)
    changed = np.count_nonzero(gray > 10)           # pixels above noise floor
    return changed / gray.size


def _feature_match_ratio(img_a: np.ndarray, img_b: np.ndarray) -> float:
    """
    ORB feature match ratio:
        ratio = good_matches / max(kp_a, kp_b)
    Returns value in [0, 1]. Close to 1.0 → nearly identical scene.
    """
    detector = cv2.ORB_create(200)

    def detect(img):
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        return detector.detectAndCompute(gray, None)

    kp_a, des_a = detect(img_a)
    kp_b, des_b = detect(img_b)

    if des_a is None or des_b is None or len(kp_a) < 4 or len(kp_b) < 4:
        return 1.0   # can't compare → assume redundant (safe default)

    matcher = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
    matches = matcher.match(des_a, des_b)
    good    = [m for m in matches if m.distance < 50]

    return len(good) / max(len(kp_a), len(kp_b))


# ── main public function ───────────────────────────────────────────────────────

def redundancy_filter(image_a: np.ndarray, image_b: np.ndarray) -> bool:
    """
    Evaluate whether image_b is redundant compared to image_a.

    Parameters
    ----------
    image_a : np.ndarray  — reference frame (BGR)
    image_b : np.ndarray  — candidate frame  (BGR)

    Returns
    -------
    True  → SKIP image_b (too similar / redundant)
    False → PASS image_b (new content, proceed to stitching)
    """

    # ── Stage 1 · Perceptual Hash (microseconds) ──────────────────────────────
    hash_a   = _phash(image_a)
    hash_b   = _phash(image_b)
    distance = _hamming(hash_a, hash_b)

    if distance <= HASH_HAMMING_THRESHOLD:
        print(f"[SKIP] Stage 1 – pHash distance={distance}  (≤{HASH_HAMMING_THRESHOLD})")
        return True                                  # identical hash → skip early

    # ── Stage 2 · Pixel Difference Ratio (milliseconds) ──────────────────────
    diff_ratio = _pixel_diff_ratio(image_a, image_b)

    if diff_ratio < PIXEL_DIFF_THRESHOLD:
        print(f"[SKIP] Stage 2 – pixel diff={diff_ratio:.3f}  (<{PIXEL_DIFF_THRESHOLD})")
        return True                                  # almost no pixels changed

    # ── Stage 3 · Feature Match Ratio (tens of milliseconds) ─────────────────
    match_ratio = _feature_match_ratio(image_a, image_b)

    if match_ratio >= FEATURE_MATCH_THRESHOLD:
        print(f"[SKIP] Stage 3 – feature match={match_ratio:.2f}  (≥{FEATURE_MATCH_THRESHOLD})")
        return True                                  # same scene, same features

    # ── All stages passed → genuinely new frame ───────────────────────────────
    print(f"[PASS] hash={distance}  diff={diff_ratio:.3f}  feat={match_ratio:.2f}")
    return False