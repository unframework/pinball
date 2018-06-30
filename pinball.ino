#include <TVout.h>
#include <fontALL.h>

TVout TV;

float ball[2] = { -20, 40 };
float ball_d[2] = { 0.2, 0 };

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

bool on = false; // strobe to detect freezes

int tvCX;
int tvCY;

struct bumper {
  int p1[2], p2[2];
  float normal[2];
  float offset;
  float left, right;
};

#define BUMPHW 12
#define MAX_BUMP_X (60 - BUMPHW)
#define BUMP(x, y, vd) { { x - BUMPHW, y + vd }, { x + BUMPHW, y - vd } }

bumper bumpers[] = {
  BUMP(-MAX_BUMP_X, 0, 1),
  BUMP(MAX_BUMP_X, 0, -2),

  BUMP(0, 35, 3),

  BUMP((MAX_BUMP_X / 2) * -1, 15, 2),
  BUMP((MAX_BUMP_X / 2) * 1, 15, -3),

  BUMP(0, 0, 2),

  BUMP((MAX_BUMP_X / 2) * -1, -15, 3),
  BUMP((MAX_BUMP_X / 2) * 1, -15, -2),

  BUMP(0, -35, -4),
};

float leftWallNormal[] = { 1, 0 };
float rightWallNormal[] = { -1, 0 };

void computeEdges(struct bumper *self) {
  float p1[2] = { self->p1[0], self->p1[1] };
  float delta[2] = { self->p2[0] - self->p1[0], self->p2[1] - self->p1[1] };
  float lengthSq = vec2dot(delta, delta);
  float length = sqrt(lengthSq);

  float along[2] = { delta[0] / length, delta[1] / length };
  self->normal[0] = -along[1];
  self->normal[1] = along[0];
  self->offset = vec2dot(self->normal, p1);

  float originDelta[2] = { self->p1[0] - self->offset * self->normal[0], self->p1[1] - self->offset * self->normal[1] };
  self->left = vec2dot(along, originDelta) - EPS; // extra length to avoid "leaks"
  self->right = self->left + length + EPS + EPS;
}

float applyBumper(float ball[], float ball_d[], float portion, struct bumper *bottom) {
  // bounce?
  float bottomPos = vec2dot(ball, bottom->normal);
  float bottomPos_rel = bottomPos - bottom->offset;
  float bottomPos_d = -vec2dot(ball_d, bottom->normal);

  // see if our (portioned) delta will "eat away" at any distance we have left
  if (bottomPos_d > 0 && bottomPos_rel >= -EPS && bottomPos_rel < portion * bottomPos_d) {
    float subPortion = bottomPos_rel / bottomPos_d;
    float alongPos = vec2cross(bottom->normal, ball) + subPortion * vec2cross(bottom->normal, ball_d);

    // check against ends
    if (alongPos > bottom->left && alongPos < bottom->right) {
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

void physicsStep() {
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

    float leftWallPortion = applyWall(ball, ball_d, travelPortion, leftWallNormal, -55);

    if (leftWallPortion < closestPortion) {
      closestPortion = leftWallPortion;
      closestBumperNormal = leftWallNormal;
    }

    float rightWallPortion = applyWall(ball, ball_d, travelPortion, rightWallNormal, -55);

    if (rightWallPortion < closestPortion) {
      closestPortion = rightWallPortion;
      closestBumperNormal = rightWallNormal;
    }

    for (struct bumper *bmp = bumpers; bmp < (struct bumper *)(&bumpers + 1); bmp += 1) {
      float portion = applyBumper(ball, ball_d, travelPortion, bmp);

      if (portion < closestPortion) {
        closestPortion = portion;
        closestBumperNormal = bmp->normal;
      }
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
    if (closestBumperNormal != 0) {
      float normalVel = vec2dot(ball_d, closestBumperNormal);

      // non-linear damping
      float dampenedAmount = 2 * normalVel + min(0.1, -normalVel);
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

  for (struct bumper *bmp = bumpers; bmp < (struct bumper *)(&bumpers + 1); bmp += 1) {
    computeEdges(bmp);
  }
}

void loop() {
  on = !on;

  TV.delay_frame(1);

  physicsStep();

  TV.clear_screen();
  TV.draw_rect(1, 1, 1, 1, on ? WHITE : BLACK);

  drawBall();
  drawBumpers();
}

void drawBall() {
  float s_ball[2] = { ball[0] + tvCX, tvCY - ball[1] };

  // screen-space sizing
  TV.draw_line(s_ball[0] - 2, s_ball[1], s_ball[0], s_ball[1] + 2, WHITE);
  TV.draw_line(s_ball[0], s_ball[1] + 2, s_ball[0] + 2, s_ball[1], WHITE);
  TV.draw_line(s_ball[0] + 2, s_ball[1], s_ball[0], s_ball[1] - 2, WHITE);
  TV.draw_line(s_ball[0], s_ball[1] - 2, s_ball[0] - 2, s_ball[1], WHITE);
}

void drawBumpers() {
  for (struct bumper *bmp = bumpers; bmp < (struct bumper *)(&bumpers + 1); bmp += 1) {
    TV.draw_line(tvCX + bmp->p1[0], tvCY - bmp->p1[1], tvCX + bmp->p2[0], tvCY - bmp->p2[1], WHITE);
  }
}
