#include <TVout.h>
#include <fontALL.h>

TVout TV;

float ball[2] = { 0, 40 };
float ball_d[2] = { 0.1, 0 };

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

bool on = false; // strobe to detect freezes

int tvCX;
int tvCY;

struct bumper {
  float normal[2];
  float offset;
  float left, right;
  float along[2];
  float p1[2], p2[2];
};

bumper bumpers[] = {
  { { -sin(-0.1), cos(-0.1) }, -10, -10, 10 },
  { { -sin(0.1), cos(0.1) }, -30, -35, 35 },
  { { -sin(1.2), cos(1.2) }, -30, -35, 35 },
  { { -sin(2.6), cos(2.6) }, -30, -35, 35 },
  { { -sin(-2.6), cos(-2.6) }, -30, -35, 35 },
  { { -sin(-1.2), cos(-1.2) }, -30, -35, 35 }
};

void computeEdges(struct bumper *self) {
  self->along[0] = self->normal[1];
  self->along[1] = -self->normal[0];

  float selfDrawOffset = self->offset - 2;
  float origin[2] = { selfDrawOffset * self->normal[0], selfDrawOffset * self->normal[1] };

  float left = self->left;
  float right = self->right;

  self->p1[0] = origin[0] + left * self->along[0];
  self->p1[1] = origin[1] + left * self->along[1];
  self->p2[0] = origin[0] + right * self->along[0];
  self->p2[1] = origin[1] + right * self->along[1];
}

float applyBumper(float ball[], float ball_d[], float portion, struct bumper *bottom) {
  // bounce?
  float bottomPos = vec2dot(ball, bottom->normal);
  float bottomPos_rel = bottomPos - bottom->offset;
  float bottomPos_d = -vec2dot(ball_d, bottom->normal);

  // see if our (portioned) delta will "eat away" at any distance we have left
  if (bottomPos_d > 0 && bottomPos_rel >= -EPS && bottomPos_rel < portion * bottomPos_d) {
    float subPortion = bottomPos_rel / bottomPos_d;
    float alongPos = vec2dot(ball, bottom->along) + subPortion * vec2dot(ball_d, bottom->along);

    // check against ends
    if (alongPos > bottom->left && alongPos < bottom->right) {
      // return portion of travel
      return subPortion;
    }
  }

  return portion;
}

void physicsStep() {
  // add gravity
  ball_d[1] -= 0.05;

  float travelPortion = 1;

  do {
    float closestPortion = travelPortion;
    struct bumper *closestBumper = 0;

    for (struct bumper *bmp = bumpers; bmp < &bumpers + 1; bmp += 1) {
      float portion = applyBumper(ball, ball_d, travelPortion, bmp);

      if (portion < closestPortion) {
        closestPortion = portion;
        closestBumper = bmp;
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
    if (closestBumper != 0) {
      float normalVel = vec2dot(ball_d, closestBumper->normal);

      // non-linear damping
      float dampenedAmount = 2 * normalVel + min(0.6, -normalVel);
      ball_d[0] -= dampenedAmount * closestBumper->normal[0];
      ball_d[1] -= dampenedAmount * closestBumper->normal[1];
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

  for (struct bumper *bmp = bumpers; bmp < &bumpers + 1; bmp += 1) {
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
  for (struct bumper *bmp = bumpers; bmp < &bumpers + 1; bmp += 1) {
    TV.draw_line(tvCX + bmp->p1[0], tvCY - bmp->p1[1], tvCX + bmp->p2[0], tvCY - bmp->p2[1], WHITE);
  }
}
