from ament_pep257.main import main
import pytest


_EXTRA_IGNORE = 'D213'


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=['.', 'test', '--add-ignore', _EXTRA_IGNORE])
    assert rc == 0, 'Found code style errors / warnings'
