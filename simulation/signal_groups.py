"""Joint programmes for adjacent signals with no usable inter-signal storage."""
from collections import defaultdict
from dataclasses import dataclass


@dataclass(frozen=True)
class PortalGroup:
    leader: str
    follower: str
    follower_phases: tuple
    cells: frozenset
    exits: frozenset


def joint_program(leader, follower, ports, reference_count, minimum_green):
    def mask(green, receiving):
        state = list(green)
        for index, lane in ports.items():
            if not any(movement.incoming == lane and follower.greens[receiving][movement.index] == "G"
                       for movement in follower.movements):
                state[index] = "r"
        return "".join(state)

    references = []
    for green, duration in zip(leader.greens[:reference_count], leader.green_durations[:reference_count]):
        choices = [phase for phase in range(len(follower.greens))
                   if any(green[index] in "gG" and mask(green, phase)[index] in "gG" for index in ports)]
        if not choices:
            choices = [0]
        for phase in choices:
            references.append([mask(green, phase), phase, max(minimum_green, duration / len(choices))])
    for index in range(len(references) - 1, -1, -1):
        green, phase, duration = references[index]
        dominant = next((other for other, (candidate, receiving, _) in enumerate(references)
                         if other != index and receiving == phase
                         and all(value == "r" or value == candidate[position] for position, value in enumerate(green))
                         and (candidate != green or other < index)), None)
        if dominant is not None:
            references[dominant][2] += duration
            references.pop(index)
    required = {movement.index for movement in leader.movements
                if any(green[movement.index] in "gG" for green in leader.greens[:reference_count])}
    covered = {index for green, _, _ in references for index, value in enumerate(green) if value in "gG"}
    if not required <= covered:
        raise ValueError("No-storage joint reference leaves a movement unserved")
    result = list(references)
    seen = {(green, phase) for green, phase, _ in result}
    for green, duration in zip(leader.greens, leader.green_durations):
        for phase in range(len(follower.greens)):
            state = mask(green, phase)
            if "g" in state.lower() and (state, phase) not in seen:
                result.append([state, phase, max(minimum_green, duration)])
                seen.add((state, phase))
    return (tuple(row[0] for row in result), tuple(row[2] for row in result),
            tuple(row[1] for row in result), len(references))


def build_portal_groups(topology, reference_counts, settings):
    candidates = defaultdict(dict)
    for key, signal in topology.signals.items():
        for movement in signal.movements:
            path = topology.receivers[movement.outgoing]
            follower = topology.owner.get(path[-1])
            if (follower in topology.signals and follower != key
                    and sum(topology.lanes[lane].length for lane in path) < settings.storage_per_vehicle):
                candidates[key, follower][movement.index] = path[-1]
    groups, assigned = {}, set()
    for (leader_id, follower_id), ports in sorted(candidates.items()):
        leader, follower = topology.signals[leader_id], topology.signals[follower_id]
        if assigned & {leader_id, follower_id} or {movement.incoming for movement in follower.movements} != set(ports.values()):
            continue
        greens, durations, targets, reference_count = joint_program(
            leader, follower, ports, reference_counts[leader_id], settings.minimum_green)
        outgoing = {movement.outgoing for movement in leader.movements if movement.index in ports}
        external = {lane for start in outgoing for lane in topology.receivers[start]}
        internal = {lane for lane, (road, _) in topology.connector_exits.items() if road in external}
        internal.update(lane for lane, (_, road) in topology.connector_movements.items() if road in outgoing)
        internal.update(topology.internal_lanes[follower_id])
        exits = {movement.outgoing for movement in follower.movements}
        groups[leader_id] = PortalGroup(leader_id, follower_id, targets, frozenset(external | internal), frozenset(exits))
        leader.greens, leader.green_durations = greens, durations
        reference_counts[leader_id] = reference_count
        assigned.update((leader_id, follower_id))
    return groups
