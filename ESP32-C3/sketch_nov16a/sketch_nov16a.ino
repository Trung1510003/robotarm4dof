// ESP32-C3 Servo Controller - MIXED SERVOS (MG996 + SG90)
// Chân 2,3,4: MG996 (1000-2000us)
// Chân 5: SG90 (500-2400us)

#include <ESP32Servo.h>

// Servo objects
Servo servo1, servo2, servo3, servo4;

// Chân GPIO
const int servo1_pin = 2;  // MG996
const int servo2_pin = 3;  // MG996
const int servo3_pin = 4;  // MG996
const int servo4_pin = 5;  // SG90

// THÔNG SỐ PWM THEO LOẠI SERVO
// MG996: 1000-2000us (hoặc thử 544-2400us nếu không đúng)
// SG90: 500-2400us
const int MG996_MIN = 1000;
const int MG996_MAX = 2000;
const int SG90_MIN = 500;
const int SG90_MAX = 2400;

// Góc hiện tại (TUYỆT ĐỐI)
float current_angles[4] = {36, 150, 100, 90};     // ← Góc mới
float previous_angles[4] = {36, 150, 100, 90};
bool servos_initialized = false;

// Thống kê
int command_count = 0;
float max_delta = 0;
float total_delta = 0;

// DEADBAND - Bỏ qua thay đổi nhỏ hơn ngưỡng này
const float DEADBAND = 0.5;  // độ (0.5° = bỏ qua thay đổi nhỏ)

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10);
  }
  
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("🤖 ESP32-C3 MIXED SERVO CONTROLLER");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  
  // Allocate timers
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  
  Serial.println("\n📋 SERVO CONFIGURATION:");
  
  // SERVO 1 - MG996 (Chân 2)
  Serial.print("Servo 1 (Pin "); Serial.print(servo1_pin); Serial.print("): MG996 ");
  Serial.print(MG996_MIN); Serial.print("-"); Serial.print(MG996_MAX); Serial.println("us");
  if (servo1.attach(servo1_pin, MG996_MIN, MG996_MAX)) {
    servo1.write(36);
    Serial.println("  ✓ Initialized at 90°");
  } else {
    Serial.println("  ✗ FAILED!");
  }
  
  // SERVO 2 - MG996 (Chân 3)
  Serial.print("Servo 2 (Pin "); Serial.print(servo2_pin); Serial.print("): MG996 ");
  Serial.print(MG996_MIN); Serial.print("-"); Serial.print(MG996_MAX); Serial.println("us");
  if (servo2.attach(servo2_pin, MG996_MIN, MG996_MAX)) {
    servo2.write(150);
    Serial.println("  ✓ Initialized at 90°");
  } else {
    Serial.println("  ✗ FAILED!");
  }
  
  // SERVO 3 - MG996 (Chân 4)
  Serial.print("Servo 3 (Pin "); Serial.print(servo3_pin); Serial.print("): MG996 ");
  Serial.print(MG996_MIN); Serial.print("-"); Serial.print(MG996_MAX); Serial.println("us");
  if (servo3.attach(servo3_pin, MG996_MIN, MG996_MAX)) {
    servo3.write(100);
    Serial.println("  ✓ Initialized at 90°");
  } else {
    Serial.println("  ✗ FAILED!");
  }
  
  // SERVO 4 - SG90 (Chân 5)
  Serial.print("Servo 4 (Pin "); Serial.print(servo4_pin); Serial.print("): SG90 ");
  Serial.print(SG90_MIN); Serial.print("-"); Serial.print(SG90_MAX); Serial.println("us");
  if (servo4.attach(servo4_pin, SG90_MIN, SG90_MAX)) {
    servo4.write(90);
    Serial.println("  ✓ Initialized at 90°");
  } else {
    Serial.println("  ✗ FAILED!");
  }
  
  servos_initialized = true;
  delay(1500);  // Chờ servo ổn định
  
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("✅ READY - ABSOLUTE ANGLE MODE");
  Serial.print("📊 Deadband: ±"); Serial.print(DEADBAND); Serial.println("°");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("Commands:");
  Serial.println("  TEST - Check connection");
  Serial.println("  START ABSOLUTE - Begin operation");
  Serial.println("  SERVO,a1,a2,a3,a4 - Move servos");
  Serial.println("  STOP - Stop and go to safe position");
  Serial.println("  STATUS - Show current angles");
  Serial.println("  STATS - Show statistics");
  Serial.println("  CALIBRATE - Test servo range");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

void loop() {
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command == "TEST") {
      Serial.println("OK-MIXED_SERVOS_READY");
      reportStatus();
    }
    else if (command == "START ABSOLUTE") {
      Serial.println("OK-ABSOLUTE_MODE_STARTED");
      command_count = 0;
      max_delta = 0;
      total_delta = 0;
      Serial.println("📊 Statistics reset. Ready to receive commands.\n");
    }
    else if (command == "STOP") {
      Serial.println("OK-STOPPING");
      reportStatistics();
      moveToSafePosition();
    }
    else if (command.startsWith("SERVO")) {
      processServoCommand(command);
    }
    else if (command == "STATUS") {
      reportStatus();
    }
    else if (command == "STATS") {
      reportStatistics();
    }
    else if (command == "CALIBRATE") {
      runCalibration();
    }
    else if (command.startsWith("TEST_JOINT")) {
      testSingleJoint(command);
    }
    else {
      Serial.println("ERROR:UNKNOWN_COMMAND");
    }
  }
}

void processServoCommand(String cmd) {
  if (!servos_initialized) {
    Serial.println("ERROR:SERVOS_NOT_INITIALIZED");
    return;
  }
  
  // Parse: "SERVO,angle1,angle2,angle3,angle4"
  int firstComma = cmd.indexOf(',');
  if (firstComma == -1) {
    Serial.println("ERROR:INVALID_FORMAT");
    return;
  }
  
  String params = cmd.substring(firstComma + 1);
  float target_angles[4];
  int angleCount = 0;
  
  int startIndex = 0;
  int commaIndex;
  
  do {
    commaIndex = params.indexOf(',', startIndex);
    String angleStr;
    if (commaIndex == -1) {
      angleStr = params.substring(startIndex);
    } else {
      angleStr = params.substring(startIndex, commaIndex);
    }
    
    target_angles[angleCount] = angleStr.toFloat();
    
    // Validate angle
    if (target_angles[angleCount] < 0 || target_angles[angleCount] > 180) {
      Serial.print("ERROR:ANGLE_OUT_OF_RANGE_J");
      Serial.print(angleCount + 1);
      Serial.print(":");
      Serial.println(target_angles[angleCount]);
      return;
    }
    
    angleCount++;
    startIndex = commaIndex + 1;
  } while (commaIndex != -1 && angleCount < 4);
  
  if (angleCount != 4) {
    Serial.print("ERROR:NEED_4_ANGLES_GOT_");
    Serial.println(angleCount);
    return;
  }
  
  // LƯU GÓC TRƯỚC
  for (int i = 0; i < 4; i++) {
    previous_angles[i] = current_angles[i];
  }
  
  // TÍNH DELTA
  float deltas[4];
  float max_delta_this_cmd = 0;
  bool has_movement = false;
  
  command_count++;
  
  // CHỈ HIỂN THỊ CHI TIẾT MỖI 50 LỆNH
  bool verbose = (command_count % 50 == 1) || (command_count <= 5);
  
  if (verbose) {
    Serial.print("━━━ CMD #"); Serial.print(command_count); Serial.println(" ━━━");
  }
  
  for (int i = 0; i < 4; i++) {
    deltas[i] = target_angles[i] - previous_angles[i];
    float abs_delta = abs(deltas[i]);
    
    // DEADBAND - Bỏ qua thay đổi quá nhỏ
    if (abs_delta < DEADBAND) {
      target_angles[i] = previous_angles[i];  // Giữ nguyên
      deltas[i] = 0;
      abs_delta = 0;
    } else {
      has_movement = true;
    }
    
    if (verbose && abs_delta > 0.01) {
      Serial.print("J"); Serial.print(i+1); Serial.print(": ");
      Serial.print(previous_angles[i], 1); Serial.print("°→");
      Serial.print(target_angles[i], 1); Serial.print("° (");
      if (deltas[i] >= 0) Serial.print("+");
      Serial.print(deltas[i], 2); Serial.print("°)");
      
      if (abs_delta > 10.0) {
        Serial.print(" ⚠️");
      }
      Serial.println();
    }
    
    if (abs_delta > max_delta_this_cmd) {
      max_delta_this_cmd = abs_delta;
    }
    
    total_delta += abs_delta;
  }
  
  if (max_delta_this_cmd > max_delta) {
    max_delta = max_delta_this_cmd;
  }
  
  // CẢNH BÁO NẾU THAY ĐỔI LỚN BẤT THƯỜNG
  if (max_delta_this_cmd > 20.0) {
    Serial.println("\n🚨 WARNING: Very large angle change!");
    Serial.print("   Max Δ = "); Serial.print(max_delta_this_cmd, 2); Serial.println("°");
    Serial.println("   → Check MATLAB angle mapping!");
  }
  
  // ĐIỀU KHIỂN SERVO (chỉ khi có chuyển động)
  if (has_movement) {
    servo1.write((int)round(target_angles[0]));
    servo2.write((int)round(target_angles[1]));
    servo3.write((int)round(target_angles[2]));
    servo4.write((int)round(target_angles[3]));
    
    // CẬP NHẬT GÓC HIỆN TẠI
    for (int i = 0; i < 4; i++) {
      current_angles[i] = target_angles[i];
    }
    
    if (verbose) {
      Serial.print("✅ Moved | MaxΔ: ");
      Serial.print(max_delta_this_cmd, 2);
      Serial.println("°");
    }
  } else {
    if (verbose) {
      Serial.println("⏸️  No movement (below deadband)");
    }
  }
  
  if (verbose) {
    Serial.println();
  }
}

void reportStatus() {
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("📊 CURRENT STATUS");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("ANGLES (ABSOLUTE):");
  Serial.print("  J1 (MG996): "); Serial.print(current_angles[0], 1); Serial.println("°");
  Serial.print("  J2 (MG996): "); Serial.print(current_angles[1], 1); Serial.println("°");
  Serial.print("  J3 (MG996): "); Serial.print(current_angles[2], 1); Serial.println("°");
  Serial.print("  J4 (SG90):  "); Serial.print(current_angles[3], 1); Serial.println("°");
  Serial.print("Commands: "); Serial.println(command_count);
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

void reportStatistics() {
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("📈 STATISTICS");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.print("Total commands: "); Serial.println(command_count);
  Serial.print("Max Δ angle: "); Serial.print(max_delta, 2); Serial.println("°");
  
  if (command_count > 0) {
    float avg_delta = total_delta / (command_count * 4);
    Serial.print("Avg Δ angle: "); Serial.print(avg_delta, 3); Serial.println("°");
  }
  
  Serial.println("\n🔍 DIAGNOSIS:");
  if (max_delta > 30) {
    Serial.println("  🚨 PROBLEM: Very large changes!");
    Serial.println("     → Check MATLAB joint_limits");
    Serial.println("     → Check servo angle mapping");
  } else if (max_delta > 10) {
    Serial.println("  ⚠️  Large changes detected");
    Serial.println("     → May be normal for trajectory");
  } else if (max_delta < 5) {
    Serial.println("  ✅ EXCELLENT: Smooth motion");
  } else {
    Serial.println("  ✅ GOOD: Normal operation");
  }
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

void moveToSafePosition() {
  Serial.println("🔄 Moving to safe position (90°,90°,90°,90°)...");
  
  servo1.write(36);
  servo2.write(150);
  servo3.write(100);
  servo4.write(90);
  
  for (int i = 0; i < 4; i++) {
    current_angles[i] = 90;
    previous_angles[i] = 90;
  }
  
  delay(1000);
  Serial.println("✅ SAFE_POSITION_REACHED\n");
}

void runCalibration() {
  Serial.println("\n━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("🔧 SERVO CALIBRATION TEST");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  Serial.println("Testing servo range...\n");
  
  int test_angles[] = {90, 0, 90, 180, 90};
  String positions[] = {"CENTER", "MIN (0°)", "CENTER", "MAX (180°)", "CENTER"};
  
  for (int i = 0; i < 5; i++) {
    int angle = test_angles[i];
    
    Serial.print("→ "); Serial.print(positions[i]);
    Serial.print(" ("); Serial.print(angle); Serial.println("°)");
    
    servo1.write(angle);
    servo2.write(angle);
    servo3.write(angle);
    servo4.write(angle);
    
    delay(1500);
    
    Serial.println("  ✓ Check servo positions!");
    delay(500);
  }
  
  Serial.println("\n✅ Calibration complete");
  Serial.println("\nIf MG996 servos don't reach correct positions:");
  Serial.println("  → Try changing MG996_MIN/MAX to 544-2400");
  Serial.println("If servos rotate continuously:");
  Serial.println("  → Wrong servo type or damaged");
  Serial.println("━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n");
}

void testSingleJoint(String cmd) {
  // Format: TEST_JOINT,joint_num,angle
  // Example: TEST_JOINT,1,45
  
  int firstComma = cmd.indexOf(',');
  int secondComma = cmd.indexOf(',', firstComma + 1);
  
  if (firstComma == -1 || secondComma == -1) {
    Serial.println("ERROR: Format: TEST_JOINT,joint(1-4),angle(0-180)");
    return;
  }
  
  int joint = cmd.substring(firstComma + 1, secondComma).toInt();
  int angle = cmd.substring(secondComma + 1).toInt();
  
  if (joint < 1 || joint > 4 || angle < 0 || angle > 180) {
    Serial.println("ERROR: Joint must be 1-4, angle 0-180");
    return;
  }
  
  Serial.print("Testing J"); Serial.print(joint);
  Serial.print(" → "); Serial.print(angle); Serial.println("°");
  
  switch (joint) {
    case 1: servo1.write(angle); Serial.println("  (MG996)"); break;
    case 2: servo2.write(angle); Serial.println("  (MG996)"); break;
    case 3: servo3.write(angle); Serial.println("  (MG996)"); break;
    case 4: servo4.write(angle); Serial.println("  (SG90)"); break;
  }
  
  current_angles[joint-1] = angle;
  Serial.println("✓ Done\n");
}