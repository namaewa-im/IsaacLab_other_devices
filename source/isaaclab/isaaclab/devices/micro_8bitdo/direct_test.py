#!/usr/bin/env python3
"""
Isaac Sim Python 환경에서 직접 실행하는 8BitDo 컨트롤러 테스트
"""

import time
import sys

def main():
    print("=" * 60)
    print("Isaac Sim Python 환경에서 8BitDo 컨트롤러 테스트")
    print("=" * 60)
    print(f"Python 버전: {sys.version}")
    print(f"Python 경로: {sys.executable}")
    
    try:
        # Isaac Sim 환경 확인
        import carb
        print("✓ carb 모듈 로드됨")
        
        # 입력 인터페이스 획득
        input_interface = carb.input.acquire_input_interface()
        print("✓ 입력 인터페이스 획득됨")
        
        # 사용 가능한 입력 장치 확인
        print("\n사용 가능한 입력 장치:")
        try:
            # 키보드 확인
            keyboard = input_interface.get_keyboard()
            if keyboard:
                print(f"✓ 키보드: {input_interface.get_keyboard_name(keyboard)}")
            else:
                print("⚠ 키보드 없음")
        except Exception as e:
            print(f"⚠ 키보드 확인 실패: {e}")
        
        try:
            # 마우스 확인
            mouse = input_interface.get_mouse()
            if mouse:
                print(f"✓ 마우스: {input_interface.get_mouse_name(mouse)}")
            else:
                print("⚠ 마우스 없음")
        except Exception as e:
            print(f"⚠ 마우스 확인 실패: {e}")
        
        # 게임패드 확인 (여러 개 가능)
        print("\n게임패드 확인:")
        for i in range(4):  # 최대 4개 게임패드 확인
            try:
                gamepad = input_interface.get_gamepad(i)
                if gamepad:
                    gamepad_name = input_interface.get_gamepad_name(gamepad)
                    print(f"✓ 게임패드 {i}: {gamepad_name}")
                    
                    # 게임패드 이벤트 테스트
                    print(f"  게임패드 {i} 이벤트 테스트 시작...")
                    
                    def on_gamepad_event(event, *args):
                        if abs(event.value) > 0.01:  # 데드존
                            print(f"  이벤트: {event.type} = {event.value:.3f}")
                    
                    # 이벤트 구독
                    subscription = input_interface.subscribe_to_gamepad_events(gamepad, on_gamepad_event)
                    print(f"  ✓ 게임패드 {i} 이벤트 구독 완료")
                    
                    # 10초간 테스트
                    print(f"  게임패드 {i}를 조작해보세요! (10초)")
                    for count in range(100):
                        time.sleep(0.1)
                        if count % 20 == 0:
                            print(f"  진행상황: {count/10:.1f}초")
                    
                    # 구독 해제
                    input_interface.unsubscribe_from_gamepad_events(gamepad, subscription)
                    print(f"  ✓ 게임패드 {i} 테스트 완료")
                    
                else:
                    print(f"⚠ 게임패드 {i}: 없음")
            except Exception as e:
                print(f"⚠ 게임패드 {i} 확인 실패: {e}")
        
        print("\n테스트 완료!")
        print("8BitDo 컨트롤러가 연결되어 있다면 위에서 이벤트가 출력되었을 것입니다.")
        
    except Exception as e:
        print(f"❌ 오류: {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    main() 