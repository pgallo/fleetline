"""End-to-end: simulator -> network reordering -> C++ core -> tabular reports."""

from fleetline import demo


def test_demo_run_produces_consistent_reports():
    r = demo.run(vehicles=5, hours=6.0, seed=3)
    st = r["engine"].stats()
    assert st["processed"] == r["points"]  # nothing late with a 30 s window and <= 25 s delays
    assert st["late_dropped"] == 0
    assert st["outliers"] > 0  # the simulator injects GPS jumps; the filter must catch some

    util = r["utilisation"]
    assert len(util) == 5
    assert (util["trips"] > 0).all()
    assert util["utilisation_pct"].between(1, 100).all()
    assert util["avg_speed_kmh"].between(10, 90).all()

    idle = r["idle"]
    assert list(idle["rank"]) == list(range(1, 6))
    assert idle["idle_min"].is_monotonic_decreasing

    dwell = r["dwell"]
    assert "depot" in dwell.index
    assert (dwell["completed"] <= dwell["visits"]).all()

    assert not r["etas"].empty
    assert (r["etas"]["eta_min"] >= 0).all()


def test_demo_is_deterministic():
    a = demo.run(vehicles=3, hours=3.0, seed=9)
    b = demo.run(vehicles=3, hours=3.0, seed=9)
    assert a["utilisation"].equals(b["utilisation"])
    assert a["dwell"].equals(b["dwell"])


def test_demo_cli_prints_reports(capsys):
    demo.main(["--vehicles", "3", "--hours", "3", "--seed", "1"])
    out = capsys.readouterr().out
    for heading in ("Utilisation per vehicle", "Idle-time ranking", "Geofence dwell report", "ETA to depot"):
        assert heading in out
    assert "points/s" in out
