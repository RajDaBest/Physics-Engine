#include "particle.h"
#include <SDL2/SDL.h>
#include <stdlib.h>
#include <stdbool.h>

#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080
#define CIRCLE_RADIUS 10
#define MAX_PARTICLES 1000 // Set a limit to avoid excessive particles

typedef struct {
    Particle particle;
    bool active; // Track if the particle is active
} ParticleInstance;

void renderCircle(SDL_Renderer *renderer, int x, int y, int radius) {
    for (int w = 0; w < radius * 2; w++) {
        for (int h = 0; h < radius * 2; h++) {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx * dx + dy * dy) <= (radius * radius)) {
                SDL_RenderDrawPoint(renderer, x + dx, y + dy);
            }
        }
    }
}

void initializeParticle(Particle *particle) {
    particle->position = vectorDef(CIRCLE_RADIUS, WINDOW_HEIGHT / 2, 0.0); // center of window
    particle->velocity = vectorDef(600, 1000, 0.0);
    particle->acceleration = vectorDef(0, -1200, 0.0);
    setMass(particle, 1.0);
    particle->damping = 0.99;
    particle->forceAccum = vectorDef(0, -1200, 0.0);
}

int main() {
    if (SDL_Init(SDL_INIT_VIDEO) < 0) return 1;
    SDL_Window *window = SDL_CreateWindow("Particle Simulation", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);

    ParticleInstance particles[MAX_PARTICLES] = {0}; // Array to hold particle instances
    int particleCount = 0; // Number of active particles

    bool running = true;
    SDL_Event event;

    while (running) {
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) running = false;
            if (event.type == SDL_KEYDOWN && event.key.keysym.sym == SDLK_a) {
                if (particleCount < MAX_PARTICLES) {
                    // Create a new particle at the center with specified attributes
                    initializeParticle(&particles[particleCount].particle);
                    particles[particleCount].active = true;
                    particleCount++;
                }
            }
        }

        // Integrate and update each active particle
        for (int i = 0; i < particleCount; i++) {
            if (particles[i].active) {
                integrate(&particles[i].particle, 0.016); // Approx. 60 FPS
                if (particles[i].particle.position.y < -CIRCLE_RADIUS) {
                    particles[i].active = false; // Deactivate particle if it goes off-screen
                }
            }
        }

        // Render
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        for (int i = 0; i < particleCount; i++) {
            if (particles[i].active) {
                int screenX = (int)particles[i].particle.position.x;
                int screenY = WINDOW_HEIGHT - (int)particles[i].particle.position.y;
                renderCircle(renderer, screenX, screenY, CIRCLE_RADIUS);
            }
        }

        SDL_RenderPresent(renderer);
        SDL_Delay(16); // 16 ms delay for 60 FPS
    }

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();
    return 0;
}
