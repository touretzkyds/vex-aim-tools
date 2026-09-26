"""Command preprocessing for Celeste open-vocabulary requests."""
import re

from .worldmap import WorldObject, OpenVocabObj


def target_commands(commands, lookup, object_types):
    """Route open-vocabulary actions through target confirmation."""
    normalize = lambda text: re.sub(r'[^a-z0-9]', '', text.lower())
    builtin_names = [normalize(name.removesuffix('Obj')) for name, value in object_types.items()
                     if isinstance(value, type) and issubclass(value, WorldObject)
                     and value not in (WorldObject, OpenVocabObj)]
    result = []
    confirmed = set()
    for command in commands:
        action, _, target = command.partition(' ')
        if action not in ('#search', '#pilottoobject', '#kick') or not target.strip():
            result.append(command)
            if action == '#find':
                confirmed.add(normalize(target.partition(':')[0]))
            continue
        target = target.strip()
        try:
            obj = lookup(target)
        except ValueError:
            obj = None
        key = normalize(target)
        builtin = obj is not None and not isinstance(obj, OpenVocabObj)
        builtin = builtin or (obj is None and any(
            key.startswith(name) or key == name.removesuffix('obj')
            for name in builtin_names if name))
        if builtin:
            result.append(command)
            continue
        if key not in confirmed and not (obj is not None and obj.is_valid and not obj.is_missing):
            result.append('#find ' + target)
            confirmed.add(key)
        if action == '#pilottoobject':
            result.append(command)
        elif action == '#kick':
            result.extend(['#pilottoobject ' + target, '#kick'])
    return result



def lookup_openvocab_target(spec, objects, pose):
    """Resolve only open-vocabulary targets; leave other specs to the navigator."""
    exact = objects.get(spec)
    if exact is not None:
        return exact if isinstance(exact, OpenVocabObj) else None
    targets = [obj for obj in objects.values()
               if isinstance(obj, OpenVocabObj) and obj.is_valid]
    if not targets:
        return None
    try:
        pattern = re.compile(spec)
        # Preserve the navigator's built-in name matching before OV fallback.
        builtin_pattern = re.compile(''.join(spec.split()))
        if any(not isinstance(obj, OpenVocabObj) and obj.is_valid
               and builtin_pattern.match(obj.name) for obj in objects.values()):
            return None
        matches = [obj for obj in targets if pattern.match(obj.name)]
        if not matches:
            squash = lambda text: re.sub(r'[\s_-]+', '', text)
            pattern = re.compile(squash(spec), re.IGNORECASE)
            matches = [obj for obj in targets if pattern.search(squash(obj.name))]
    except re.error:
        return None
    return min(matches, key=lambda obj: (obj.pose.x - pose.x)**2 +
               (obj.pose.y - pose.y)**2, default=None)
