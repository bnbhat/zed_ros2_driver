import os
import yaml
import pyzed.sl as sl

BODY_FORMAT_INTEGER = {
    sl.BODY_FORMAT.BODY_18 : 18,
    sl.BODY_FORMAT.BODY_34 : 34,
    sl.BODY_FORMAT.BODY_38 : 38
} 

class Config:
    """Configuration for the ZED ROS2 Driver"""
    def __init__(self, filename="config.yaml") -> None:
        self.filename = filename
        self.root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.raw_config = self.load_config()
        self.build_config()

    def build_config(self) -> None:
        self.resolution  = getattr(sl.RESOLUTION, self.raw_config["resolution"])
        self.body_format = getattr(sl.BODY_FORMAT, self.raw_config["skeleton_type"])
        self.body_format_int_value = BODY_FORMAT_INTEGER[self.body_format]
        self.detection_model = getattr(sl.BODY_TRACKING_MODEL, self.raw_config["detection_model"])
        self.Tcw = self.raw_config["Tcw"]
        self.detection_confidence = self.raw_config["detection_confidence"]
        self.input_svo_file = None
        self.ip_address = None

    def get_root_dir(self) -> str:
        return self.root_dir

    def load_config(self) -> dict | None:
        """Load the config from the config.yaml file
            default path: root_dir/config/config.yaml
        """
        file_path = os.path.join(self.root_dir, 'config', self.filename)
        with open(file_path, 'r') as stream:
            try:
                return yaml.safe_load(stream)
            except FileNotFoundError as exc:
                print(exc)
                print("[Config Builder] Config file not found, Using default values")
                return None
            except yaml.YAMLError as exc:
                print(exc)
                print("[Config Builder] Error loading config file, Using default values")
                return None
    
    def override(self, args):
        """Override the config with the arguments passed from the command line"""
        if args.input_svo_file:
            self.input_svo_file = args.input_svo_file
        
        if args.ip_address:
            self.config["ip_address"] = args.ip_address

        if args.resolution:
            self.config["resolution"] = getattr(sl.RESOLUTION, [args.resolution])

class ConfigBuilder:
    """Builds the configuration for the ZED ROS2 Driver"""
    @staticmethod
    def get_config(filename="config.yaml", args=None) -> Config:
        config = Config(filename)
        if args:
            config.override(args)
        return config


if __name__ == "__main__":
    config = ConfigBuilder.get_config()
    print(config.resolution)
    print(config.body_format)
    print(config.body_format_int_value)
    print(config.detection_model)
    print(config.Tcw)
    print(config.detection_confidence)
    print(config.input_svo_file)
    print(config.ip_address)
    print(type(config.resolution))
    print(type(sl.RESOLUTION.HD720))