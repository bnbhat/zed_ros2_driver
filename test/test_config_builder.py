import pytest
import yaml
import pyzed.sl as sl
from zed_driver.config_builder import Config, ConfigBuilder


# Mock data
SAMPLE_CONFIG_YAML = """
resolution: HD720
skeleton_type: BODY_18
detection_model: MODEL_0
Tcw: [1, 2, 3, 4]
detection_confidence: 0.8
"""

class MockArgs:
    def __init__(self, input_svo_file=None, ip_address=None, resolution=None):
        self.input_svo_file = input_svo_file
        self.ip_address = ip_address
        self.resolution = resolution

def test_load_config(mocker):
    mocker.patch("builtins.open", mocker.mock_open(read_data=SAMPLE_CONFIG_YAML))
    config = Config()
    assert config.resolution == sl.RESOLUTION.HD720

def test_override():
    args = MockArgs(input_svo_file="test.svo", ip_address="192.168.1.1", resolution="HD2K")
    config = ConfigBuilder.get_config(args=args)
    assert config.input_svo_file == "test.svo"
    assert config.ip_address == "192.168.1.1"
    # Add other assertions for overridden attributes

def test_config_values():
    config = ConfigBuilder.get_config()
    assert config.body_format == sl.BODY_FORMAT.BODY_18
    assert config.body_format_int_value == 18
    # Add other assertions

# Handle FileNotFoundError and yaml.YAMLError cases in load_config
def test_config_file_not_found(mocker):
    mocker.patch("os.path.join", return_value="invalid_path")
    with pytest.raises(FileNotFoundError):
        config = Config()

def test_yaml_error(mocker):
    mocker.patch("builtins.open", mocker.mock_open(read_data="not: - valid: yaml"))
    with pytest.raises(yaml.YAMLError):
        config = Config()

