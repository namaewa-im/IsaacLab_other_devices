#!/usr/bin/env python3
"""
Isaac Sim Kit 앱에서 실행하는 8BitDo 컨트롤러 테스트
"""

import time
import sys

def main():
    print("=" * 60)
    print("Isaac Sim Kit 앱에서 8BitDo 컨트롤러 테스트")
    print("=" * 60)
    
    # 파일에 로그 기록
    with open("/tmp/isaaclab_kit_test.log", "w") as f:
        f.write("IsaacLab Kit 테스트 로그 시작\n")
        f.write(f"시간: {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.flush()
        
        try:
            # Isaac Sim 환경 확인
            import carb
            f.write("✓ carb 모듈 로드됨\n")
            f.flush()
            print("✓ carb 모듈 로드됨")
            
            # 입력 인터페이스 획득
            input_interface = carb.input.acquire_input_interface()
            f.write("✓ 입력 인터페이스 획득됨\n")
            f.flush()
            print("✓ 입력 인터페이스 획득됨")
            
            # Isaac Sim의 Kit 앱에서만 사용 가능한 모듈들
            try:
                import omni.kit.app
                f.write("✓ omni.kit.app 모듈 로드됨\n")
                f.flush()
                print("✓ omni.kit.app 모듈 로드됨")
                
                # 앱 윈도우 획득
                app_window = omni.kit.app.get_app().get_main_window()
                f.write("✓ 앱 윈도우 획득됨\n")
                f.flush()
                print("✓ 앱 윈도우 획득됨")
                
                # 게임패드 확인
                gamepad = app_window.get_gamepad(0)
                if gamepad:
                    gamepad_name = input_interface.get_gamepad_name(gamepad)
                    f.write(f"✓ 게임패드 연결됨: {gamepad_name}\n")
                    f.flush()
                    print(f"✓ 게임패드 연결됨: {gamepad_name}")
                else:
                    f.write("⚠ 게임패드가 연결되지 않음\n")
                    f.flush()
                    print("⚠ 게임패드가 연결되지 않음")
                    print("8BitDo 컨트롤러를 USB로 연결해주세요")
                    return
                
                print("\n컨트롤러를 조작해보세요!")
                print("조작법:")
                print("- 왼쪽 스틱: X, Y 이동")
                print("- 오른쪽 스틱: 회전")
                print("- A 버튼: 그리퍼 토글")
                print("- B 버튼: Z 위로")
                print("- X 버튼: Z 아래로")
                print("- D-Pad: 미세 조정")
                
                # 이벤트 핸들러
                def on_event(event, *args):
                    if abs(event.value) > 0.01:
                        msg = f"이벤트: {event.type} = {event.value:.3f}"
                        print(msg)
                        f.write(msg + "\n")
                        f.flush()
                
                # 이벤트 구독
                subscription = input_interface.subscribe_to_gamepad_events(gamepad, on_event)
                f.write("✓ 이벤트 구독 완료\n")
                f.flush()
                print("✓ 이벤트 구독 완료")
                
                # 메인 루프
                count = 0
                while count < 300:  # 30초 대기
                    time.sleep(0.1)
                    count += 1
                    if count % 50 == 0:  # 5초마다 진행상황 출력
                        print(f"진행상황: {count/10:.1f}초 경과")
                        f.write(f"진행상황: {count/10:.1f}초 경과\n")
                        f.flush()
                    
            except Exception as e:
                error_msg = f"❌ Kit 앱 오류: {e}"
                print(error_msg)
                f.write(error_msg + "\n")
                f.flush()
                
        except Exception as e:
            error_msg = f"❌ 오류: {e}"
            print(error_msg)
            f.write(error_msg + "\n")
            f.flush()
            import traceback
            f.write(traceback.format_exc() + "\n")
            f.flush()
    
    print("\n테스트 완료")
    print("로그 파일: /tmp/isaaclab_kit_test.log")

if __name__ == "__main__":
    main() 