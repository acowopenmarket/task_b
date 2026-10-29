"""Pure timestamp matching; no ROS or hardware imports."""


def timestamp(message):
    return message.header.stamp.sec+message.header.stamp.nanosec*1e-9


def match_buffered(buffers, keys, seen, now, cfg):
    """Match the newest complete bundle, allowing delayed companion callbacks.

    Always trying only the newest color can starve matching when other topics
    trail it by one callback/frame. Older still-fresh colors remain eligible.
    Timestamps and all configured freshness limits are kept unchanged.
    """
    if not buffers.get('color') or any(not buffers.get(k) for k in keys):
        return None, None
    best=None
    latest_used=max(seen) if seen else float('-inf')
    for image,arrival in reversed(buffers['color']):
        if timestamp(image)<=latest_used:
            continue
        pairs={k:min(buffers[k],key=lambda x:abs(timestamp(x[0])-timestamp(image))) for k in keys}
        stamps=[timestamp(image)]+[timestamp(p[0]) for p in pairs.values()]
        span=max(stamps)-min(stamps)
        diagnostic=dict(span_s=span,image_age_s=now-arrival,
            stamp_offsets_s={k:timestamp(v[0])-timestamp(image) for k,v in pairs.items()},
            queue_sizes={k:len(v) for k,v in buffers.items()})
        robot_age=now-pairs['robot'][1] if 'robot' in pairs else None
        diagnostic['robot_arrival_age_s']=robot_age
        if best is None or span<best['span_s']:
            best=diagnostic
        if span>cfg['max_sync_s'] or not 0<=now-arrival<=cfg['max_image_age_s']:
            continue
        if robot_age is not None and not 0<=robot_age<=cfg['max_robot_age_s']:
            continue
        return (image,pairs,span),diagnostic
    return None,best
