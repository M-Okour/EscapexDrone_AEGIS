import os

from exd_system import host_stats


def test_read_temperatures(tmp_path):
    zone = tmp_path / 'thermal_zone0'
    zone.mkdir()
    (zone / 'type').write_text('CPU-therm\n')
    (zone / 'temp').write_text('45500\n')
    assert host_stats.read_temperatures(str(tmp_path)) == {'CPU-therm': 45.5}


def test_read_memory(tmp_path):
    f = tmp_path / 'meminfo'
    f.write_text('MemTotal:       8000000 kB\nMemAvailable:   2000000 kB\n')
    mem = host_stats.read_memory(str(f))
    assert round(mem['used_percent']) == 75


def test_missing_files_are_ok(tmp_path):
    assert host_stats.read_temperatures(str(tmp_path / 'nope')) == {}
    assert host_stats.read_memory(os.path.join(str(tmp_path), 'nope')) == {}
