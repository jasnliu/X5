import json,sys,hashlib
from pathlib import Path
p=Path(__file__).resolve().parent
h=json.loads((p/'hypotheses.json').read_text()); c=json.loads((p/'comparisons.json').read_text()); r=json.loads((p/'robustness.json').read_text()); t=json.loads((p/'timing_grid.json').read_text())
verification={}
for label in ('source','session'):
 unchanged=[];changed=[]
 for line in (p/(label+'_before.sha256')).read_text().splitlines():
  expected,name=line.split('  ',1)
  (unchanged if hashlib.sha256(Path(name).read_bytes()).hexdigest()==expected else changed).append(name)
 if label=='source': assert not changed, changed
 else:
  assert all(name.endswith('/can_query_audit.jsonl') or name.endswith('/orbbec_sdk_logs/OrbbecSDK.log.txt') for name in changed),changed
 verification[label+'_files_hash_verified_unchanged']=len(unchanged)
 if changed:verification['existing_user_capture_process_continues_appending_runtime_logs']=changed
session=Path('calibration_sessions/20261017T211949Z-762f9908'); s=json.loads((session/'session.json').read_text())
assert s['fit'] is None and s['validation'] is None and not (session/'calibration.json').exists()
verification.update(no_calibration_applied=True,no_calibration_export_created=True,no_hardware_access_by_diagnostics=True,validation_poses_not_used=True)
report=dict(
 conclusion='Dominant systematic encoder-to-URDF kinematic calibration mismatch; small middle-joint zero/axis-alignment biases are the strongest tested explanation.',
 certainty='Strong, repeatable statistical diagnosis from saved data. Exact physical encoder zeros versus mechanically equivalent axis/alignment errors are not independently identifiable from these observations alone.',
 source_session=str(session.resolve()),training_samples=25,reserved_validation_samples=5,
 original_training_metrics=h['baseline']['metrics'],
 diagnostic_joint_offset_model_metrics=h['joints_2_to_6_offsets']['metrics'],
 internal_five_fold_crossvalidation=dict(original=r['baseline_5fold']['metrics'],joint_offset_hypothesis=r['joint_offsets_5fold']['metrics']),
 diagnostic_only_joint_offset_estimates_degrees=h['joints_2_to_6_offsets']['joint_offsets_deg'],
 estimate_warning='Not a calibrated motor-setting prescription; no offset, URDF, capture policy or threshold has been changed. J1/J7 constants are absorbed by free extrinsics and were not estimated.',
 evidence=[
  'All 25 training images redetected identical corner coordinates and poses to the saved observations.',
  'Independent batched URDF FK reproduces saved hand transforms to below 1e-12.',
  'Joint-offset hypothesis improves both positional and angular consistency and passes the original aggregate thresholds in internal five-fold prediction.',
  'Estimating from early 15 poses predicts late 10 better, and estimating from late 15 predicts early 10 better.',
  'Rotation-only diagnostic independently estimates similar middle-joint biases; scale/link lengths alone cannot explain the angular inconsistency.',
  'Printed scale, anisotropic print scale, simple fixed board bow, simple gravity-induced mount flex, and intrinsic adjustment models did not match the predictive improvement.',
  'A 41-point uniform timing-offset sweep across saved encoder history did not materially reduce errors.',
  'The factory distortion handling is specified Brown-Conrady K6. Ignoring distortion does not resolve the failure.'
 ],
 predictive_pixel_comparison={k:dict(fitted_rms_px=c[k]['full_rms'],internal_cv_rms_px=c[k]['crossvalidated_rms']) for k in ('baseline','joint_offsets','camera_K','board_scale','board_anisotropic','board_bow','mount_gravity_flex')},
 timing_best=t['best'],rotation_only_check=r['rotation_only'],early_late_prediction_checks={k:r[k] for k in ('early_to_late','late_to_early')},
 limitations=[
  'No independent dimensional measurement or mechanical zero reference is available in this offline dataset.',
  'Other pose-dependent mechanical errors can be partly absorbed into fitted zero offsets; exact mechanical cause is not proven.',
  'Residual calibration, physical accuracy, and collision safety have not been validated.',
  'Reserved first-five validation images/measurements were not evaluated or optimized against.',
  'The SDK 3D-to-2D helper returned essentially undistorted pinhole pixels in a separate arithmetic check; it is not used by this RGB calibration solver, and this check does not establish physical distortion accuracy.'
 ],verification=verification)
(p/'diagnosis_report.json').write_text(json.dumps(report,indent=2)+'\n')
print(json.dumps(dict(conclusion=report['conclusion'],verification=verification,internal_cv=report['internal_five_fold_crossvalidation']),indent=2))
