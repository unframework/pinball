#include <TVout.h>
#include <fontALL.h>

TVout TV;

struct ball_movement {
  float position[2];
  float delta[2];
};

#define ORIGIN_X -30
#define ORIGIN_Y 65

ball_movement balls[10];

float EPS = 0.00001;

void vec2add(float out[], float a[], float b[]) {
  out[0] = a[0] + b[0];
  out[1] = a[1] + b[1];
}

void vec2sub(float out[], float a[], float b[]) {
  out[0] = a[0] - b[0];
  out[1] = a[1] - b[1];
}

void vec2copy(float out[], float a[]) {
  out[0] = a[0];
  out[1] = a[1];
}

void vec2scale(float out[], float a[], float k) {
  out[0] = a[0] * k;
  out[1] = a[1] * k;
}

float vec2dot(float a[], float b[]) { return a[0] * b[0] + a[1] * b[1]; }

float vec2cross(float a[], float b[]) { return a[1] * b[0] - a[0] * b[1]; }

int tvCX;
int tvCY;

int frameCount = 0;
#define MAX_FRAME_COUNT 6000 // a thousand frames is about half a minute?

struct bumperCircle {
  float center[2];
  float radius;
};

float ball_d_angle = 0;

float circleRadius = 30;
float circleOffset_box[] = {20, 25};

float angle_unit[2];
float angle_unit_cross[2];

unsigned int bumpCount = 0;

bumperCircle mainBumperCircle = {{0, 0}, 10};

float leftWallNormal[] = {1, 0};
float rightWallNormal[] = {-1, 0};
float bottomWallNormal[] = {0, 1};

void resetEnvironment() {
  ball_d_angle = M_PI * random(-10000, 10000) * 0.0001;

  angle_unit[0] = cos(ball_d_angle);
  angle_unit[1] = sin(ball_d_angle);
  angle_unit_cross[0] = -angle_unit[1];
  angle_unit_cross[1] = angle_unit[0];

  vec2scale(mainBumperCircle.center, angle_unit,
            circleRadius + circleOffset_box[0] * 0.3);

  float box[2];
  vec2scale(box, angle_unit,
            circleOffset_box[0] * random(-10000, 10000) * 0.0001);
  vec2add(mainBumperCircle.center, mainBumperCircle.center, box);

  vec2scale(box, angle_unit_cross,
            circleOffset_box[1] * random(-10000, 10000) * 0.0001);
  vec2add(mainBumperCircle.center, mainBumperCircle.center, box);

  mainBumperCircle.radius = circleRadius;
}

void resetBall(float ball[], float ball_d[]) {
  // spawn ball on a line towards center, offscreen
  vec2scale(ball, angle_unit, -70 - random(0, 10000) * 0.0001 * 40);

  // jitter the initial speed
  float box[2];
  float boost =
      0.22 * (angle_unit[1] + 1); // accelerate slightly if coming from below
  vec2scale(ball_d, angle_unit, 0.12 + boost);

  vec2scale(box, angle_unit, 0.08 * random(-10000, 10000) * 0.0001);
  vec2add(ball_d, ball_d, box);

  vec2scale(box, angle_unit_cross, 0.35 * random(-10000, 10000) * 0.0001);
  vec2add(ball_d, ball_d, box);

  ball_d[1] +=
      0.1 * abs(angle_unit[0]); // aim a bit up to compensate for gravity
}

// inspired by
// stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
float applyBumperCircle(float ball[], float ball_d[], float portion,
                        struct bumperCircle *circle) {
  float f[2];
  vec2sub(f, ball, circle->center);

  float a = vec2dot(ball_d, ball_d);
  float b = 2 * vec2dot(f, ball_d);
  float c = vec2dot(f, f) - circle->radius * circle->radius;

  float discriminant = b * b - 4 * a * c;

  if (discriminant <= 0) {
    return portion;
  }

  float t1 = (-b - sqrt(discriminant)) / (2 * a);

  if (t1 >= -0.1 && t1 < portion) {
    return t1;
  }

  return portion;
}

float applyWall(float ball[], float ball_d[], float portion, float normal[],
                float offset) {
  // bounce?
  float bottomPos = vec2dot(ball, normal);
  float bottomPos_rel = bottomPos - offset;
  float bottomPos_d = -vec2dot(ball_d, normal);

  // see if our (portioned) delta will "eat away" at any distance we have left
  if (bottomPos_d > 0 && bottomPos_rel < portion * bottomPos_d) {
    // return portion of travel
    return bottomPos_rel / bottomPos_d;
  }

  return portion;
}

void physicsStep(float ball[], float ball_d[], bool *hadCollision) {
  // default collision result
  *hadCollision = false;

  // add gravity
  ball_d[1] -= 0.001;

  if (ball[1] <= -150) {
    resetBall(ball, ball_d);
  }

  float travelPortion = 1;

  float nextX = ball[0] + ball_d[0];
  float nextY = ball[1] + ball_d[1];

  int iterCount = 0;
  do {
    iterCount += 1;

    float closestPortion = travelPortion;
    float *closestBumperNormal = 0;
    struct bumperCircle *closestBumperCircle = 0;

    // float leftWallPortion = applyWall(ball, ball_d, travelPortion,
    // leftWallNormal, -56);

    // if (leftWallPortion < closestPortion) {
    //   closestPortion = leftWallPortion;
    //   closestBumperNormal = leftWallNormal;
    // }

    // float rightWallPortion = applyWall(ball, ball_d, travelPortion,
    // rightWallNormal, -56);

    // if (rightWallPortion < closestPortion) {
    //   closestPortion = rightWallPortion;
    //   closestBumperNormal = rightWallNormal;
    // }

    // float bottomWallPortion = applyWall(ball, ball_d, travelPortion,
    // bottomWallNormal, -40);

    // if (bottomWallPortion < closestPortion) {
    //   closestPortion = bottomWallPortion;
    //   closestBumperNormal = bottomWallNormal;
    // }

    float mod1BumperCirclePortion =
        applyBumperCircle(ball, ball_d, travelPortion, &mainBumperCircle);

    if (mod1BumperCirclePortion < closestPortion) {
      closestPortion = mod1BumperCirclePortion;
      closestBumperCircle = &mainBumperCircle;
    }

    // snip away portion we can travel
    ball[0] += closestPortion * ball_d[0];
    ball[1] += closestPortion * ball_d[1];

    travelPortion -= closestPortion;

    // hacky solution to being computationally "stuck"
    // @todo improve?
    if (closestPortion == 0) {
      travelPortion -= 0.05;
    }

    // apply bumper restitution (circles are done last, so they are checked
    // first)
    if (closestBumperCircle != 0) {
      float normal[2];

      vec2sub(normal, ball, closestBumperCircle->center);
      float normalLen = sqrt(vec2dot(normal, normal));
      normal[0] /= normalLen;
      normal[1] /= normalLen;

      // @todo make reusable function
      float normalVel = vec2dot(ball_d, normal);

      // non-linear damping
      float dampenedAmount = 2 * normalVel + min(0.2, -normalVel);
      ball_d[0] -= dampenedAmount * normal[0];
      ball_d[1] -= dampenedAmount * normal[1];

      *hadCollision = true;
    } else if (closestBumperNormal != 0) {
      float normalVel = vec2dot(ball_d, closestBumperNormal);

      // non-linear damping
      float dampenedAmount =
          2 * normalVel +
          min(closestBumperNormal[1] == 0 ? -0.2 : 0.12, -normalVel);
      ball_d[0] -= dampenedAmount * closestBumperNormal[0];
      ball_d[1] -= dampenedAmount * closestBumperNormal[1];

      *hadCollision = true;
    }
  } while (travelPortion > 0 && iterCount < 20);
}

void setup() {
  pinMode(4, OUTPUT);

  TV.begin(NTSC, 120, 96);

  tvCX = TV.hres() / 2;
  tvCY = TV.vres() / 2;

  randomSeed(analogRead(0));

  frameCount = MAX_FRAME_COUNT; // trigger initial reset
}

void loop() {
  if (frameCount < MAX_FRAME_COUNT) {
    frameCount += 1;

    TV.delay_frame(1);

    int processedCount =
        1 + frameCount / 100; // restrict how many balls are processed at first
    bool anyCollision = false;

    for (struct ball_movement *ball = balls;
         ball < (struct ball_movement *)(&balls + 1); ball += 1) {
      // bail out early if needed
      processedCount -= 1;
      if (processedCount <= 0) {
        break;
      }

      // drawBall(ball->position);
      bool ballHadCollision;
      physicsStep(ball->position, ball->delta, &ballHadCollision);
      drawBall(ball->position); // @todo consider proper lines

      anyCollision = anyCollision || ballHadCollision;
    }

    // "render" tick sound
    if (anyCollision) {
      digitalWrite(4, bumpCount % 2);
      bumpCount += 1; // update bump count to alternate speaker value
    }
  } else {
    // produce a noticeable signal reset and re-initialize display
    TV.end();

    delay(500);

    TV.begin(NTSC, 120, 96);

    tvCX = TV.hres() / 2;
    tvCY = TV.vres() / 2;

    // set up scene
    resetEnvironment();

    for (struct ball_movement *ball = balls;
         ball < (struct ball_movement *)(&balls + 1); ball += 1) {
      resetBall(ball->position, ball->delta);
    }

    frameCount = 0;
  }
}

void drawBall(float ball[]) {
  // perform crude clipping to avoid uint8 overflow
  if (abs(ball[0]) < 100 && abs(ball[1]) < 100) {
    float s_ball[2] = {ball[0] + tvCX, tvCY - ball[1]};

    TV.set_pixel(s_ball[0], s_ball[1], INVERT);
  }
}
