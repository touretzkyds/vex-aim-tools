"""Recognition and verification prompts, separate from API calls and robot state."""
import json


def verification_prompt(label, identity_context, tolerance=0):
    """Build the active prompt; nonzero tolerance is an opt-in experiment."""
    question = (f'The robot is looking for "{label}". The top panel is the camera image. '
                'Each numbered panel below is a crop of that same image with one candidate box '
                'in green. Judge only the object outlined by candidate 1, not other objects '
                'visible elsewhere in the full scene or crop. A target elsewhere in the image '
                'does not make this candidate a match. Return JSON only: {"objects": [{"boxes": [numbers of the candidates '
                'that outline this physical object or part of it], "name": what this object is, '
                '"target": true if that name is what the robot is looking for or a model of it, '
                '"grounded": true only if one of these boxes has its bottom edge where the object '
                'touches the surface}], "reason": short explanation}. List each physical object '
                'once. A box that also covers much of the background does not outline the object. '
                'Judge cut-off objects from the top panel, not the crop edges: an object cut off '
                'at the left, right or bottom of the image is not grounded.'
                + identity_context)
    if tolerance:
        question = question.replace(
            '"grounded": true only if one of these boxes has its bottom edge where the object '
            'touches the surface',
            '"grounded": true if the visible supporting base lies within the amber tolerance band')
        question += (
            f' Ground-contact tolerance: the two amber lines mark {tolerance} original-image '
            'pixels above and below the green box bottom. Exact alignment is not required. '
            'Set grounded=true when the visible nearest supporting base/contact falls within '
            'this band and the box remains a reasonable outline of this candidate. '
            'Feet, rounded or uneven bases, and gaps between legs are valid; the whole '
            'bottom edge and its midpoint need not touch physical material. Top clipping '
            'alone does not invalidate a visible base. Keep grounded=false for a missing '
            'or occluded base, substantial bottom offset, or left/right/bottom image clipping. '
            'Shadows are not the base. Identity must independently match the requested '
            'object; the band never excuses a wrong identity. Add a base_reason field '
            'to each object: usable, missing_base, clipped_base, offset, or uncertain. '
            'If the base cannot be judged, return grounded=false and uncertain.'
        )
    return question


def recognition_prompt(label, identity_context, max_variants, max_variant_chars):
    return (
        'Is this object in the robot camera image, on the surface the robot is on: ' +
        json.dumps(label) + '? Return JSON only: {"presence": "present", "absent" or '
        '"uncertain", "variants": up to ' + str(max_variants) + ' short phrases an '
        'open-vocabulary detector such as YOLOE could use for it, '
        '"candidate_edge": "left", "right" '
        'or null, "reason": short explanation}. Use present only if you can identify the '
        'object. Set candidate_edge only if one possible match is cut off at that side of '
        "the image. For variants, use the object's name, an alternate name or close "
        'rephrasing, and a distinguishing visual description supported by the image. '
        'Keep every phrase specific to the requested target. Avoid broad categories '
        'that could equally describe nearby objects. '
        f'Each phrase must be at most {max_variant_chars} characters, including spaces. '
        'Use concise wording that preserves distinguishing features.' + identity_context)


def batch_verification_prompt(label, identity_context, tolerance=0):
    """Build the multi-candidate prompt without model or robot dependencies."""
    question = (
        'Find the requested object among the numbered green-box crops below the full scene. '
        'Judge the object inside each box, not a different object elsewhere in the scene. '
        'Select at most one candidate. Prefer a correct identity with a usable base. '
        'Grounded means the bottom box edge reaches the visible contact with the supporting surface. '
        'Missing or occluded bases, substantial bottom offset, and left/right/bottom image clipping '
        'are not grounded. Top clipping alone is allowed. Shadows are not the base. '
        'If no candidate matches, return candidate=null. If identity or base judgment needs '
        'a closer examination, select the most plausible candidate and set uncertain=true. '
        'Return JSON {"candidate": integer or null, "grounded": boolean, '
        '"uncertain": boolean, "reason": "explanation"}. Requested object: '
        + json.dumps(label) + identity_context)
    if tolerance:
        question = question.replace(
            'Grounded means the bottom box edge reaches the visible contact with the supporting surface.',
            'Grounded means the visible supporting base lies within the amber tolerance band.')
        question += (
            f' The amber lines mark {tolerance} original-image pixels above and below '
            'the green box bottom. Exact alignment is not required. Feet, rounded or '
            'uneven bases, and gaps between legs are valid; the bottom midpoint need '
            'not touch physical material. Identity must still match. Missing, occluded '
            'or image-clipped bases remain unusable; shadows are not bases.')
    return question
