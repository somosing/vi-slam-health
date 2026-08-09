from vif_eval.failure import evaluate_failures


def test_failure_and_recovery_metrics(tmp_path):
    events = tmp_path / "events.csv"
    labels = tmp_path / "labels.csv"
    events.write_text(
        "timestamp_sec,event,attempt_id\n"
        "10.2,REQUESTED,1\n"
        "12.0,SUCCEEDED,1\n"
        "40.0,REQUESTED,2\n",
        encoding="utf-8",
    )
    labels.write_text(
        "start_sec,end_sec,label\n10.0,11.0,blackout\n25.0,27.0,imu_dropout\n",
        encoding="utf-8",
    )
    result = evaluate_failures(events, labels)
    assert result["true_positives"] == 1
    assert result["false_positives"] == 1
    assert result["false_negatives"] == 1
    assert result["precision"] == 0.5
    assert result["recall"] == 0.5
    assert result["recovery_attempts"] == 2
    assert result["recovery_successes"] == 1
    assert abs(result["recovery_time_mean_sec"] - 1.8) < 1e-9
