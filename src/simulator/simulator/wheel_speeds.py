from loguru import logger as llogger


def translate_u8(wheel_speed_u8: int) -> float:
    if wheel_speed_u8 < 126:
        return float(-wheel_speed_u8)
    elif wheel_speed_u8 == 126:
        return 0.0
    else:  # its gt 126
        # complain if it's more than 255, the max value
        if wheel_speed_u8 > 255:
            llogger.error(
                "converted wheel speed above 255! that shouldn't happen. giving it 255."
            )
            return 255.0
        return float(wheel_speed_u8) - 126.0
