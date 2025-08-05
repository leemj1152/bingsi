import rclpy
import DR_init

# 기본 설정
ROBOT_ID = "dsr01"
ROBOT_MODEL = "m0609"
VELOCITY, ACC = 40, 40
DR_init.__dsr__id = ROBOT_ID
DR_init.__dsr__model = ROBOT_MODEL

rclpy.init()
node = rclpy.create_node("box_grip_release", namespace=ROBOT_ID)
DR_init.__dsr__node = node

from DSR_ROBOT2 import movej, movel, mwait, get_current_posx, set_digital_output, wait

ON, OFF = 1, 0


# ====== 그립/릴리즈 ======
def grip():
    set_digital_output(1, OFF)
    set_digital_output(2, ON)
    wait(1)

def release():
    set_digital_output(2, ON)
    set_digital_output(1, ON)
    wait(1)


# ====== Box 클래스 ======
class Box:
    def __init__(self, box_id, position_id, target_commands):
        self.box_id = box_id
        self.position_id = position_id
        self.target_commands = target_commands  # list of dicts

    def info(self):
        info_text = f"\n[Box: {self.box_id}] Position ID: {self.position_id}\n"
        for i, cmd in enumerate(self.target_commands):
            info_text += f"  {i+1}: ({cmd['type']}) {cmd['pose']}\n"
        return info_text


# ====== 이동 함수 ======
def move_to_pose(command):
    move_type = command["type"]
    pose = command["pose"]
    print(f"🔵 {move_type.upper()} 이동 중: {pose}")
    if move_type == "movej":
        movej(pose, vel=VELOCITY, acc=ACC)
    elif move_type == "movel":
        movel(pose, vel=VELOCITY, acc=ACC)
    else:
        print(f"❌ 알 수 없는 move type: {move_type}")
        return
    mwait()

def grip_and_place(pick_cmd, place_cmd):
    move_to_pose(pick_cmd)
    grip()
    mwait()
    move_to_pose(place_cmd)
    release()
    mwait()
    print("✅ 그립 → 이동 → 릴리즈 완료")

def restore_positions(box: Box):
    cmds = box.target_commands
    if len(cmds) < 2:
        print("❌ 최소 2개 이상의 위치가 필요합니다.")
        return
    print(f"🔁 {box.box_id} 복구 시작 (역순으로)")
    for i in range(len(cmds) - 1, 0, -1):
        pick = cmds[i]
        place = cmds[i - 1]
        print(f"↩️ {i}단계: {pick['pose']} → {place['pose']}")
        grip_and_place(pick, place)
    print("✅ 복구 완료")


# ====== 메인 루프 ======
def main():
    box_dict = {}

    while rclpy.ok():
        print("\n====== Box Manager ======")
        print("1. 박스 등록")
        print("2. 박스 목록 보기")
        print("3. 박스 작업 실행 (그립 후 릴리즈)")
        print("4. 박스 삭제 및 역복구 수행")
        print("5. 종료")
        print("==========================")
        cmd = input(">> ")

        if cmd == "1":
            box_id = input("📦 Box ID >> ")
            position_id = input("🪧 Position ID >> ")
            print("📍 이동 명령 입력 (예: [{'type': 'movej', 'pose': [...]}, {'type': 'movel', 'pose': [...]}])")
            target_commands = input(">> ").strip().replace('\n', '')
            try:
                target_commands = eval(target_commands)
                assert isinstance(target_commands, list)
                for cmd in target_commands:
                    assert isinstance(cmd, dict)
                    assert "type" in cmd and cmd["type"] in ["movej", "movel"]
                    assert "pose" in cmd and isinstance(cmd["pose"], list)
            except Exception:
                print("❌ 올바른 포맷이 아닙니다.")
                continue
            box_dict[box_id] = Box(box_id, position_id, target_commands)
            print("✅ 박스 등록 완료")

        elif cmd == "2":
            if not box_dict:
                print("❌ 등록된 박스가 없습니다.")
                continue
            for box in box_dict.values():
                print(box.info())

        elif cmd == "3":
            box_id = input("실행할 박스 ID >> ")
            if box_id not in box_dict:
                print("❌ 해당 박스가 없습니다.")
                continue
            box = box_dict[box_id]
            if len(box.target_commands) < 2:
                print("❌ 최소 2개의 위치가 필요합니다.")
                continue
         #   grip_and_place(box.target_commands[0], box.target_commands[1])
            for i, cmd in enumerate(box.target_commands):
                print(f"🔁 [{i+1}/{len(box.target_commands)}] 명령 실행")
                move_to_pose(cmd)

        elif cmd == "4":
            box_id = input("삭제할 박스 ID >> ")
            if box_id not in box_dict:
                print("❌ 해당 박스가 없습니다.")
                continue
            box = box_dict[box_id]
            restore_positions(box)
            box_dict.pop(box_id)
            print(f"🗑️ {box_id} 삭제 완료")

        elif cmd == "5":
            print("🛑 종료합니다.")
            break

        else:
            print("❌ 유효하지 않은 명령입니다.")

    rclpy.shutdown()


if __name__ == "__main__":
    main()


[{"type": "movej", "pose": [14.43, 36.23, 29.37, -1.31, 112.99, 15.95]},{"type": "movej", "pose": [21.26, -1.16, 67.83, 1.68, 113.96, 21.68]},{"type": "movel", "pose": [566.35, 145.44, 224.06, 156.38, -178.23, -171.65]}]