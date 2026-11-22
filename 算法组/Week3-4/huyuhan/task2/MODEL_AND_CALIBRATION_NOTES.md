# Final Model and Calibration Notes

## Model selection

The expanded_candidate checkpoint has the highest
reported mAP50-95 among the inspected training records.

expanded_candidate:
mAP50-95 = 0.29265
mAP50 = 0.64355
Precision = 0.59337
Recall = 0.59492
Recorded epoch = 8

candidate_from81_960:
mAP50-95 = 0.24660
Precision = 0.64569
Recall = 0.53026

formal_v1:
mAP50-95 = 0.22134
Baseline checkpoint.

Other candidate checkpoints and their training records
are retained for transparency.

Important:
These values originate from individual results.csv files.
Validation splits may differ. They are not claimed as a
unified independent test comparison.

## Calibration

See calibration/README.md.

## Scope

ROS2 offline replay and Huber IRLS experiments are
independent bonus studies.

TensorRT was not implemented.

No claim of current live-camera verification is made.
