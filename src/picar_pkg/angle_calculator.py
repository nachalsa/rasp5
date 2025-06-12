import pytest
from src.picar_pkg.angle_calculator import calculate_new_angle

def test_calculate_angle_normal():
    """일반적인 경우 테스트"""
    assert calculate_new_angle(90, 10) == 100

def test_calculate_angle_with_negative_value():
    """음수 값이 들어왔을 때 0을 반환하는지 테스트"""
    assert calculate_new_angle(10, -20) == 0

def test_calculate_angle_exceeding_max():
    """최대값(180)을 넘었을 때 180을 반환하는지 테스트"""
    assert calculate_new_angle(170, 20) == 180

def test_with_invalid_type():
    """문자열 같은 잘못된 타입이 들어왔을 때 에러가 발생하는지 테스트"""
    with pytest.raises(TypeError):
        calculate_new_angle(90, "a")
