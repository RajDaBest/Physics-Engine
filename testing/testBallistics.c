#include "../src/particle.h"
#include <SDL2/SDL.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080
#define CIRCLE_RADIUS 10
#define MAX_PARTICLES 1000
#define TARGET_FPS 60
#define FRAME_TIME (1.0 / TARGET_FPS)

typedef struct
{
    Particle *particle;
    bool active;
} ParticleInstance;

// Global force parameters
static DragCoefficients dragCoeffs = {0.05, 0.005};

static void renderCircle(SDL_Renderer *renderer, int x, int y, int radius)
{
    for (int w = 0; w < radius * 2; w++)
    {
        for (int h = 0; h < radius * 2; h++)
        {
            int dx = radius - w;
            int dy = radius - h;
            if ((dx * dx + dy * dy) <= (radius * radius))
            {
                SDL_RenderDrawPoint(renderer, x + dx, y + dy);
            }
        }
    }
}

static Particle *createParticle(void)
{
    // Random velocity variation for more interesting motion
    real velX = 200 + (rand() % 50);
    real velY = 200 + (rand() % 80);

    Vector initPos = vectorDef(CIRCLE_RADIUS, WINDOW_HEIGHT / 2, 0.0);
    Vector initVel = vectorDef(velX, velY, 0.0);
    Vector initAcc = vectorDef(0.0, 0.0, 0.0); // Let forces handle acceleration

    Particle *particle = Particle_Create(
        initPos, // Initial position
        initVel, // Initial velocity
        initAcc, // Initial acceleration
        2.0,     // Mass
        0.99,    // Damping
        0.0      // Start time
    );


    if (particle)
    {
        // Add gravity force (always active)
        Particle_AddForce(particle, Particle_GravityForce, 0.0, INFINITY, NULL);

        // Add drag force (always active)
        Particle_AddForce(particle, Particle_DragForce, 0.0, INFINITY, &dragCoeffs);


    }

    return particle;
}

static void cleanupParticles(ParticleInstance *particles, int count)
{
    for (int i = 0; i < count; i++)
    {
        if (particles[i].particle)
        {
            Particle_Destroy(particles[i].particle);
            particles[i].particle = NULL;
        }
    }
}

int main(void)
{
    if (SDL_Init(SDL_INIT_VIDEO) < 0)
    {
        fprintf(stderr, "SDL initialization failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *window = SDL_CreateWindow(
        "Particle Simulation",
        SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
        WINDOW_WIDTH, WINDOW_HEIGHT,
        SDL_WINDOW_SHOWN);

    if (!window)
    {
        fprintf(stderr, "Window creation failed: %s\n", SDL_GetError());
        SDL_Quit();
        return 1;
    }

    SDL_Renderer *renderer = SDL_CreateRenderer(
        window, -1,
        SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);

    if (!renderer)
    {
        fprintf(stderr, "Renderer creation failed: %s\n", SDL_GetError());
        SDL_DestroyWindow(window);
        SDL_Quit();
        return 1;
    }

    // Initialize random seed
    srand((unsigned int)time(NULL));

    ParticleInstance particles[MAX_PARTICLES] = {0};
    int particleCount = 0;
    bool running = true;
    SDL_Event event;

    Uint32 lastTime = SDL_GetTicks();

    while (running)
    {
        // Handle events
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
            else if (event.type == SDL_KEYDOWN)
            {
                if (event.key.keysym.sym == SDLK_a && particleCount < MAX_PARTICLES)
                {
                    // Create new particle
                    Particle *newParticle = createParticle();
                    if (newParticle)
                    {
                        particles[particleCount].particle = newParticle;
                        particles[particleCount].active = true;
                        particleCount++;
                    }
                }
            }
        }

        // Calculate delta time
        Uint32 currentTime = SDL_GetTicks();
        real deltaTime = 1.0 / 60.0;
        lastTime = currentTime;

        // Update physics
        for (int i = 0; i < particleCount; i++)
        {
            if (particles[i].active && particles[i].particle)
            {
                Particle_Integrate(particles[i].particle, deltaTime);

                // Check if particle is off screen
                if (particles[i].particle->position.y < -CIRCLE_RADIUS)
                {
                    particles[i].active = false;
                    Particle_Destroy(particles[i].particle);
                    particles[i].particle = NULL;
                }
            }
        }

        // Render
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        SDL_SetRenderDrawColor(renderer, 255, 0, 0, 255);
        for (int i = 0; i < particleCount; i++)
        {
            if (particles[i].active && particles[i].particle)
            {
                int screenX = (int)particles[i].particle->position.x;
                int screenY = WINDOW_HEIGHT - (int)particles[i].particle->position.y;
                renderCircle(renderer, screenX, screenY, CIRCLE_RADIUS);
            }
        }

        SDL_RenderPresent(renderer);
    }

    // Cleanup
    cleanupParticles(particles, particleCount);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}