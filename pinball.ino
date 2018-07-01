#include <TVout.h>
#include <fontALL.h>

TVout TV;

struct ball_movement {
  float position[2];
  float delta[2];
};

ball_movement balls[] = {
  { { -20, 40 }, { 0.2, 0 } },
  { { 10, 30 }, { 0.5, 0 } },
  { { -10, 20 }, { 0.1, 0 } },
  { { 20, -10 }, { -0.3, 0 } }
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

#define LEFT_ANGLE (2 * M_PI / 3)
#define RIGHT_ANGLE (M_PI / 3)

float leftNormal[] = { cos(LEFT_ANGLE), sin(LEFT_ANGLE) };
float rightNormal[] = { cos(RIGHT_ANGLE), sin(RIGHT_ANGLE) };

#define BUMPER_OUTER(block) { \
  { float* bumperNormal = leftNormal; float bumperHalfWidth = 5 / bumperNormal[1]; int bumperNudge = 10; block } \
  { float* bumperNormal = rightNormal; float bumperHalfWidth = 5 / bumperNormal[1]; int bumperNudge = -10; block } \
}

#define BUMPER_LOOP(block) for (int j = -1; j <= 1; j += 1) { \
  float vOffset = j * 32; \
  \
  for (int i = 0; i < 4; i += 1) { \
    float hOffset = -60 + (120 + bumperNudge + 30 * i + j * 15) % 120; \
    \
    float bumperOffset = hOffset * bumperNormal[0] + vOffset * bumperNormal[1]; \
    float bumperMiddle = hOffset * bumperNormal[1] - vOffset * bumperNormal[0]; \
    float bumperLeft = -bumperHalfWidth + bumperMiddle; \
    float bumperRight = bumperHalfWidth + bumperMiddle; \
    \
    block \
  } \
}


float leftWallNormal[] = { 1, 0 };
float rightWallNormal[] = { -1, 0 };

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

  float nextY = ball[1] + ball_d[1];

  while (nextY <= -48) {
    ball[1] += 96; // @todo this better
    nextY += 96;
  }

  while (nextY > 48) {
    ball[1] -= 96;
    nextY -= 96;
  }

  do {
    float closestPortion = travelPortion;
    float *closestBumperNormal = 0;

    float leftWallPortion = applyWall(ball, ball_d, travelPortion, leftWallNormal, -59);

    if (leftWallPortion < closestPortion) {
      closestPortion = leftWallPortion;
      closestBumperNormal = leftWallNormal;
    }

    float rightWallPortion = applyWall(ball, ball_d, travelPortion, rightWallNormal, -59);

    if (rightWallPortion < closestPortion) {
      closestPortion = rightWallPortion;
      closestBumperNormal = rightWallNormal;
    }

    BUMPER_OUTER({
      float ballOffset = vec2dot(ball, bumperNormal);
      float ballOffset_d = -vec2dot(ball_d, bumperNormal);
      float ballAcross = vec2cross(bumperNormal, ball);
      float ballAcross_d = vec2cross(bumperNormal, ball_d);

      BUMPER_LOOP({
        float portion = applyBumper(ballOffset, ballOffset_d, ballAcross, ballAcross_d, travelPortion, bumperOffset, bumperLeft, bumperRight);

        if (portion < closestPortion) {
          closestPortion = portion;
          closestBumperNormal = bumperNormal;
        }
      })
    })

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
    if (closestBumperNormal != 0) {
      float normalVel = vec2dot(ball_d, closestBumperNormal);

      // non-linear damping
      float dampenedAmount = 2 * normalVel + min(0.2, -normalVel);
      ball_d[0] -= dampenedAmount * closestBumperNormal[0];
      ball_d[1] -= dampenedAmount * closestBumperNormal[1];
    }
  } while(travelPortion > 0);
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
  drawBumpers();

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
  TV.draw_rect(s_ball[0], s_ball[1], 1, 1, INVERT); // strobe to detect freezes
  // TV.draw_line(s_ball[0] - 2, s_ball[1], s_ball[0], s_ball[1] + 2, WHITE);
  // TV.draw_line(s_ball[0], s_ball[1] + 2, s_ball[0] + 2, s_ball[1], WHITE);
  // TV.draw_line(s_ball[0] + 2, s_ball[1], s_ball[0], s_ball[1] - 2, WHITE);
  // TV.draw_line(s_ball[0], s_ball[1] - 2, s_ball[0] - 2, s_ball[1], WHITE);
}

void drawBumper(float bumperNormal[], float bumperOffset, float bumperLeft, float bumperRight) {
  float along[] = { bumperNormal[1], -bumperNormal[0] };
  float origin[2] = { bumperOffset * bumperNormal[0], bumperOffset * bumperNormal[1] };

  int p1[2], p2[2];

  p1[0] = origin[0] + bumperLeft * along[0];
  p1[1] = origin[1] + bumperLeft * along[1];
  p2[0] = origin[0] + bumperRight * along[0];
  p2[1] = origin[1] + bumperRight * along[1];

  TV.draw_line(tvCX + p1[0], tvCY - p1[1], tvCX + p2[0], tvCY - p2[1], INVERT);
}

void drawBumpers() {
  BUMPER_OUTER({
    BUMPER_LOOP({
      drawBumper(bumperNormal, bumperOffset, bumperLeft, bumperRight);
    });
  })
}
