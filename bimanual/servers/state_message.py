from dataclasses import dataclass
from typing import Tuple

@dataclass
class ControllerState:
    left_x: bool
    left_y: bool
    left_menu: bool
    left_thumbstick: bool
    left_index_trigger: float
    left_hand_trigger: float
    left_thumbstick_axes: Tuple[float, float]
    left_local_position: Tuple[float, float, float]
    left_local_rotation: Tuple[float, float, float, float]

    right_a: bool
    right_b: bool
    right_menu: bool
    right_thumbstick: bool
    right_index_trigger: float
    right_hand_trigger: float
    right_thumbstick_axes: Tuple[float, float]
    right_local_position: Tuple[float, float, float]
    right_local_rotation: Tuple[float, float, float, float]

def parse_controller_state(to_string_output: str) -> ControllerState:
    left_data, right_data = to_string_output.split('|')

    left_data = left_data.split(';')[1:-1]
    right_data = right_data.split(';')[1:-1]
    from IPython import embed; embed()
    
    def parse_bool(val: str) -> bool:
        return val.split(':')[1].lower() == "true"
    
    def parse_float(val: str) -> float:
        return float(val.split(':')[1])
    
    def parse_list_float(val: str) -> Tuple[float, float]:
        return list(map(float, val.split(':')[1].split(',')))
    
    def parse_list_float_3(val: str) -> Tuple[float, float, float]:
        return list(map(float, val.split(':')[1].split(',')))
    
    def parse_list_float_4(val: str) -> Tuple[float, float, float, float]:
        return list(map(float, val.split(':')[1].split(',')))

    def parse_section(data: list) -> Tuple:
        return (
            parse_bool(data[0]),
            parse_bool(data[1]),
            parse_bool(data[2]),
            parse_bool(data[3]),

            parse_float(data[4]),
            parse_float(data[5]),

            parse_list_float(data[6]),
            parse_list_float_3(data[7]),
            parse_list_float_4(data[8])
        )
    
    left_parsed = parse_section(left_data)
    right_parsed = parse_section(right_data)

    return ControllerState(*left_parsed, *right_parsed)


if __name__ == "__main__":
    test_msg = "Left Controller:;  Left X: False;  Left Y: False;  Left Menu: False;  Left Thumbstick: False;  Left Index Trigger: 0;  Left Hand Trigger: 0;  Left Thumbstick Axes: 0,0;  Left Local Position: 0,1.583851,-0.055;  Left Local Rotation: 0,0,0,1;|Right Controller:;  Right A: False;  Right B: False;  Right Menu: False;  Right Thumbstick: False;  Right Index Trigger: 0;  Right Hand Trigger: 0;  Right Thumbstick Axes: 0,0;  Right Local Position: -0.6467661,0.7493631,0.03285643;  Right Local Rotation: -0.1810999,-0.4977325,0.3899082,0.753284;"
    controller_state = parse_controller_state(test_msg)
    from IPython import embed; embed()