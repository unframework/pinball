#include <TVout.h>
#include <fontALL.h>

TVout TV;

struct ball_movement {
  float position[2];
  float delta[2];
};

ball_movement balls[] = {
  { { 0, 20 }, { 0.2, 0.4 } },
  { { 0, 20 }, { 0.5, -0.2 } },
  { { 0, 20 }, { -0.1, 0.1 } },
  { { 0, 20 }, { -0.3, -0.2 } },
  { { 0, 20 }, { 0.6, 0.2 } },
  { { 0, 20 }, { 0.7, -0.4 } },
  { { 0, 20 }, { -0.6, 0.6 } },
  { { 0, 20 }, { -0.4, -0.3 } }
};

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

float vec2dot(float a[], float b[]) {
  return a[0] * b[0] + a[1] * b[1];
}

float vec2cross(float a[], float b[]) {
  return a[1] * b[0] - a[0] * b[1];
}

int tvCX;
int tvCY;

#define LEFT_ANGLE (3 * M_PI / 4)
#define RIGHT_ANGLE (M_PI / 4)

float leftNormal[] = { cos(LEFT_ANGLE), sin(LEFT_ANGLE) };
float rightNormal[] = { cos(RIGHT_ANGLE), sin(RIGHT_ANGLE) };

#define BUMPER_OUTER(block) { \
  { float* bumperNormal = leftNormal; float bumperHalfWidth = 3 / bumperNormal[1]; int bumperNudge = 12; block } \
  { float* bumperNormal = rightNormal; float bumperHalfWidth = 3 / bumperNormal[1]; int bumperNudge = -12; block } \
}

#define BUMPER_LOOP(block) for (int j = -1; j <= 1; j += 1) { \
  float vOffset = j * 32; \
  float vOffsetX = vOffset * bumperNormal[0]; \
  float vOffsetY = vOffset * bumperNormal[1]; \
  \
  for (int i = 0; i < 4; i += 1) { \
    float hOffset = -60 + (120 + bumperNudge + 30 * i + j * 15) % 120; \
    \
    float bumperOffset = hOffset * bumperNormal[0] + vOffsetY; \
    float bumperMiddle = hOffset * bumperNormal[1] - vOffsetX; \
    float bumperLeft = -bumperHalfWidth + bumperMiddle; \
    float bumperRight = bumperHalfWidth + bumperMiddle; \
    \
    block \
  } \
}

struct bumperCircle {
  float center[2];
  float radius;
};

float leftWallNormal[] = { 1, 0 };
float rightWallNormal[] = { -1, 0 };
float bottomWallNormal[] = { 0, 1 };

float applyBumper(float ballOffset, float ballOffset_d, float ballAcross, float ballAcross_d, float portion, float bumperOffset, float bumperLeft, float bumperRight) {
  // bounce?
  float bottomPos_rel = ballOffset - bumperOffset;

  // see if our (portioned) delta will "eat away" at any distance we have left
  if (ballOffset_d > 0 && bottomPos_rel >= -EPS && bottomPos_rel < portion * ballOffset_d) {
    float subPortion = bottomPos_rel / ballOffset_d;
    float alongPos = ballAcross + subPortion * ballAcross_d;

    // check against ends
    if (alongPos > bumperLeft && alongPos < bumperRight) {
      // return portion of travel
      return subPortion;
    }
  }

  return portion;
}

// inspired by stackoverflow.com/questions/1073336/circle-line-segment-collision-detection-algorithm
float applyBumperCircle(float ball[], float ball_d[], float portion, struct bumperCircle *circle) {
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

  if (t1 >= -EPS && t1 < portion) {
    return t1;
  }

  return portion;
}

float applyWall(float ball[], float ball_d[], float portion, float normal[], float offset) {
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

void physicsStep(float ball[], float ball_d[]) {
  // add gravity
  ball_d[1] -= 0.05;

  float travelPortion = 1;

  float nextX = ball[0] + ball_d[0];
  float nextY = ball[1] + ball_d[1];

  while (nextY <= -48) {
    ball[1] += 96; // @todo this better
    nextY += 96;
  }

  while (nextY > 48) {
    ball[1] -= 96;
    nextY -= 96;
  }

  int iterCount = 0;
  do {
    iterCount += 1;

    float closestPortion = travelPortion;
    float *closestBumperNormal = 0;
    struct bumperCircle *closestBumperCircle = 0;

    float leftWallPortion = applyWall(ball, ball_d, travelPortion, leftWallNormal, -56);

    if (leftWallPortion < closestPortion) {
      closestPortion = leftWallPortion;
      closestBumperNormal = leftWallNormal;
    }

    float rightWallPortion = applyWall(ball, ball_d, travelPortion, rightWallNormal, -56);

    if (rightWallPortion < closestPortion) {
      closestPortion = rightWallPortion;
      closestBumperNormal = rightWallNormal;
    }

    // float bottomWallPortion = applyWall(ball, ball_d, travelPortion, bottomWallNormal, -40);

    // if (bottomWallPortion < closestPortion) {
    //   closestPortion = bottomWallPortion;
    //   closestBumperNormal = bottomWallNormal;
    // }

    // BUMPER_OUTER({
    //   float ballOffset = vec2dot(ball, bumperNormal);
    //   float ballOffset_d = -vec2dot(ball_d, bumperNormal);
    //   float ballAcross = vec2cross(bumperNormal, ball);
    //   float ballAcross_d = vec2cross(bumperNormal, ball_d);

    //   BUMPER_LOOP({
    //     float portion = applyBumper(ballOffset, ballOffset_d, ballAcross, ballAcross_d, travelPortion, bumperOffset, bumperLeft, bumperRight);

    //     if (portion < closestPortion) {
    //       closestPortion = portion;
    //       closestBumperNormal = bumperNormal;
    //     }
    //   })
    // })

    bumperCircle mod1BumperCircle = { { 10 * floor(ball[0] / 10) + 5, 10 * floor(ball[1] / 10) + 5 }, 2 };
    bumperCircle mod2BumperCircle = { { 10 * floor((ball[0] - 5) / 10) + 10, 10 * floor((ball[1] - 5) / 10) + 10 }, 2 };

    float mod1BumperCirclePortion = applyBumperCircle(ball, ball_d, travelPortion, &mod1BumperCircle);

    if (mod1BumperCirclePortion < closestPortion) {
      closestPortion = mod1BumperCirclePortion;
      closestBumperCircle = &mod1BumperCircle;
    }

    float mod2BumperCirclePortion = applyBumperCircle(ball, ball_d, travelPortion, &mod2BumperCircle);

    if (mod2BumperCirclePortion < closestPortion) {
      closestPortion = mod2BumperCirclePortion;
      closestBumperCircle = &mod2BumperCircle;
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

    // apply bumper restitution
    // apply bumper restitution (circles are done last, so they are checked first)
    if (closestBumperCircle != 0) {
      float normal[2];

      vec2sub(normal, ball, closestBumperCircle->center);
      float normalLen = sqrt(vec2dot(normal, normal));
      normal[0] /= normalLen;
      normal[1] /= normalLen;

      // @todo make reusable function
      float normalVel = vec2dot(ball_d, normal);

      // non-linear damping
      float dampenedAmount = 2 * normalVel + min(0.9, -normalVel);
      ball_d[0] -= dampenedAmount * normal[0];
      ball_d[1] -= dampenedAmount * normal[1];
    } else if (closestBumperNormal != 0) {
      float normalVel = vec2dot(ball_d, closestBumperNormal);

      // non-linear damping
      float dampenedAmount = 2 * normalVel + min(closestBumperNormal[1] == 0 ? -0.2 : 0.8, -normalVel);
      ball_d[0] -= dampenedAmount * closestBumperNormal[0];
      ball_d[1] -= dampenedAmount * closestBumperNormal[1];
    }
  } while(travelPortion > 0 && iterCount < 20);
}

void setup() {
  TV.begin(NTSC,120,96);

  tvCX = TV.hres() / 2;
  tvCY = TV.vres() / 2;

  TV.select_font(font8x8);
  TV.print(16,40,"PINBALL TIME");
  TV.delay(1000);

  randomSeed(analogRead(0));

  TV.clear_screen();
  // drawBumpers();

  // initial display for inverted draw to work
  for (struct ball_movement *ball = balls; ball < (struct ball_movement *)(&balls + 1); ball += 1) {
    drawBall(ball->position);
  }
}

void loop() {
  TV.delay_frame(1);

  for (struct ball_movement *ball = balls; ball < (struct ball_movement *)(&balls + 1); ball += 1) {
    drawBall(ball->position);
    physicsStep(ball->position, ball->delta);
    drawBall(ball->position);
  }

  TV.draw_rect(1, 1, 1, 1, INVERT); // strobe to detect freezes
}

void drawBall(float ball[]) {
  float s_ball[2] = { ball[0] + tvCX, tvCY - ball[1] };

  // screen-space sizing
  TV.draw_rect(s_ball[0], s_ball[1], 1, 1, INVERT);
  // TV.draw_line(s_ball[0] - 2, s_ball[1], s_ball[0], s_ball[1] + 2, WHITE);
  // TV.draw_line(s_ball[0], s_ball[1] + 2, s_ball[0] + 2, s_ball[1], WHITE);
  // TV.draw_line(s_ball[0] + 2, s_ball[1], s_ball[0], s_ball[1] - 2, WHITE);
  // TV.draw_line(s_ball[0], s_ball[1] - 2, s_ball[0] - 2, s_ball[1], WHITE);
}

void drawBumper(int hOffset, int vOffset, float bumperNormal[], float bumperOffset, float bumperLeft, float bumperRight) {
  int p1[2], p2[2];

  // super hacky but more precise display for 45-degree line
  int amt = bumperNormal[0] > 0 ? -3 : 3;
  p1[0] = hOffset - 3;
  p1[1] = vOffset - amt;
  p2[0] = hOffset + 3;
  p2[1] = vOffset + amt;

  TV.draw_line(tvCX + p1[0], tvCY - p1[1], tvCX + p2[0], tvCY - p2[1], INVERT);
}

void drawBumpers() {
  BUMPER_OUTER({
    BUMPER_LOOP({
      drawBumper(hOffset, vOffset, bumperNormal, bumperOffset, bumperLeft, bumperRight);
    });
  })
}
