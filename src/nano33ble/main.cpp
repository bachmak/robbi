#include <Arduino.h>
#include <Arduino_LSM9DS1.h>
#include <MadgwickAHRS.h>
#include <Servo.h>

#include <array>

void rotate_async(Servo &servo, int position_deg)
{
    position_deg = std::min(position_deg, 180);
    position_deg = std::max(position_deg, 0);
    servo.write(position_deg);
}

float get_distance(int trig_pin, int echo_pin)
{
    // turn off
    digitalWrite(trig_pin, LOW);
    delayMicroseconds(2);

    // generate signal
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trig_pin, LOW);

    auto duration = pulseIn(echo_pin, HIGH, 1000);
    auto distance = duration * 3.4f / 2.0f; // us -> m

    return distance;
}

struct USContext
{
    struct MeasurementPosition
    {
        int adjusted;
        int reported;
    };

    Servo servo;
    int last_position_idx = 0;

    const int servo_pin = 12;
    const int trig_pin = 8;
    const int echo_pin = 9;

    uint32_t last_update_time_ms = 0;
    const uint32_t update_interval_ms = 600;

    const std::array<MeasurementPosition, 4> measurement_positions = {
        {
            {0, 180},
            {80, 90},
            {170, 0},
            {80, 90},
        }};
};

struct IMUContext
{
    Madgwick filter;

    const float offset_gx = 1.83;
    const float offset_gy = -0.75;
    const float offset_gz = -0.30;

    const float offset_ax = 0.005;
    const float offset_ay = -0.018;
    const float offset_az = 0.018;

    const float mag_off_x = 5.97;
    const float mag_off_y = 41.04;
    const float mag_off_z = -5.65;

    const float mag_slope_x = 1.19;
    const float mag_slope_y = 0.87;
    const float mag_slope_z = 0.99;

    const int filter_frequency = 15;

    uint32_t last_update_time_ms = 0;
    const uint32_t update_interval_ms = 50;
};

struct Context
{
    USContext ultrasonic;
    IMUContext imu;
};

void init(USContext &ctx)
{
    pinMode(ctx.trig_pin, OUTPUT);
    pinMode(ctx.trig_pin, OUTPUT);
    pinMode(ctx.echo_pin, INPUT);

    ctx.servo.attach(ctx.servo_pin);
    ctx.servo.write(0);
}

void update(USContext &ctx)
{
    const auto current_time_ms = millis();
    if (current_time_ms < ctx.last_update_time_ms + ctx.update_interval_ms)
    {
        return;
    }

    const auto distance = get_distance(ctx.trig_pin, ctx.echo_pin);

    const auto pos = ctx.measurement_positions[ctx.last_position_idx];
    rotate_async(ctx.servo, pos.adjusted);

    ctx.last_update_time_ms = current_time_ms;
    ctx.last_position_idx += 1;
    ctx.last_position_idx %= static_cast<int>(ctx.measurement_positions.size());

    char buf[32];

    snprintf(buf, sizeof(buf), "us %d %.3f\n", pos.reported, distance);
    Serial.write(buf);
}

void init(IMUContext &ctx)
{
    if (!IMU.begin())
    {
        Serial.println("Failed to initialize IMU!");
        while (1)
            ;
    }

    IMU.setMagnetOffset(ctx.mag_off_x, ctx.mag_off_y, ctx.mag_off_z);
    IMU.setMagnetSlope(ctx.mag_slope_x, ctx.mag_slope_y, ctx.mag_slope_z);

    ctx.filter.begin(ctx.filter_frequency);
}

void update(IMUContext &ctx)
{
    if (!IMU.accelerationAvailable() ||
        !IMU.gyroscopeAvailable() ||
        !IMU.magneticFieldAvailable())
    {
        return;
    }

    const auto current_time_ms = millis();
    if (current_time_ms < ctx.last_update_time_ms + ctx.update_interval_ms)
    {
        return;
    }

    ctx.last_update_time_ms = current_time_ms;

    float ax, ay, az;
    float gx, gy, gz;
    float mx, my, mz;
    float roll, pitch, yaw;

    IMU.readAcceleration(ax, ay, az);
    IMU.readGyroscope(gx, gy, gz);
    IMU.readMagneticField(mx, my, mz);

    ax += ctx.offset_ax;
    ay += ctx.offset_ay;
    az += ctx.offset_az;

    gx += ctx.offset_gx;
    gy += ctx.offset_gy;
    gz += ctx.offset_gz;

    ctx.filter.update(gx, gy, gz, ax, ay, az, mx, my, mz);

    yaw = ctx.filter.getYaw();
    pitch = ctx.filter.getPitch();
    roll = ctx.filter.getRoll();

    // Loop delay to control data rate
    char buf[32];

    snprintf(buf, sizeof(buf), "imu %.2f %.2f %.2f\n", yaw, pitch, roll);
    Serial.write(buf);
}

void init(Context &ctx)
{
    init(ctx.ultrasonic);
    init(ctx.imu);
}

void update(Context &ctx)
{
    update(ctx.ultrasonic);
    update(ctx.imu);
}

void setup() {}

void loop()
{
    auto ctx = Context{};

    init(ctx.ultrasonic);
    init(ctx.imu);

    Serial.begin(115200);
    delay(500);

    while (1)
    {
        update(ctx);
    }
}