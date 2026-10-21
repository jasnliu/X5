"""Human-readable safety refusals; never modifies plans or their verdicts."""


def blockage_summary(result):
    if result.get('status') == 'planned':
        return ''
    lines = ['NO TRAJECTORY — ' + str(result.get('error', result.get('details', result.get('status'))))]
    labels = {
        'body_or_clearance_buffer_too_near_or_behind_camera': 'inside near-depth cutoff',
        'body_or_clearance_buffer_outside_depth_fov': 'outside depth FOV',
        'missing_or_invalid_depth': 'missing depth',
        'observed_surface_occludes_required_clearance': 'occluded clearance',
    }
    incomplete = False
    coverage = result.get('visibility_coverage', {})
    for pose in ('start', 'goal'):
        if pose not in coverage:
            continue
        report = coverage[pose]
        count = report['links_with_unobserved_clearance']
        incomplete |= count > 0
        kinds = [label for reason, label in labels.items()
                 if any(link.get('failure_counts', {}).get(reason, 0) for link in report['links'])]
        lines.append(f'{pose.upper()}: {count}/{report["moving_links_checked"]} moving links '
                     'have unverified camera clearance' + (': ' + ', '.join(kinds) if kinds else '') + '.')
    if incomplete:
        lines.append('Safety refusal, not a camera connection failure. The camera does not establish '
                     'clearance for the whole arm and stick. Improve measured scene coverage; '
                     'recalibrate if the camera is moved. Unseen space has NOT been marked clear.')
    return '\n'.join(lines)
