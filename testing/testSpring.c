#include "../src/particle.h"
#include <SDL2/SDL.h>
#include <stdlib.h>
#include <stdbool.h>
#include <time.h>

#define WINDOW_WIDTH 1920
#define WINDOW_HEIGHT 1080
#define CIRCLE_RADIUS 10
#define MAX_PARTICLES 1000
#define TARGET_FPS 120
#define FRAME_TIME (1.0 / TARGET_FPS)

// Spring constants
#define SPRING_COEFFICIENT 50.0f
#define SPRING_REST_LENGTH 100.0f
#define SPRING_MAX_LENGTH 10000.0f

typedef struct
{
    Particle *particle;
    bool active;
    int pairIndex; // Index of the paired particle (-1 if none)
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

static void renderSpring(SDL_Renderer *renderer, Vector p1, Vector p2, int windowHeight)
{
    // Convert physics coordinates to screen coordinates
    int x1 = (int)p1.x;
    int y1 = windowHeight - (int)p1.y;
    int x2 = (int)p2.x;
    int y2 = windowHeight - (int)p2.y;

    SDL_RenderDrawLine(renderer, x1, y1, x2, y2);
}

static Vector getRandomPosition(void)
{
    return vectorDef(
        CIRCLE_RADIUS + (rand() % (WINDOW_WIDTH - 2 * CIRCLE_RADIUS)),
        CIRCLE_RADIUS + (rand() % (WINDOW_HEIGHT - 2 * CIRCLE_RADIUS)),
        0.0);
}

static Vector getRandomVelocity(void)
{
    // Random velocity between -200 and 200 in both directions
    return vectorDef(
        -200.0f + (rand() % 401),
        -200.0f + (rand() % 401),
        0.0);
}

static ParticleError createParticlePair(ParticleInstance *particles, int *particleCount)
{
    if (*particleCount >= MAX_PARTICLES - 1)
        return PARTICLE_ERROR_INVALID_PARAM;

    // Create first particle
    Vector pos1 = getRandomPosition();
    Vector vel1 = nullVectorDef();
    Vector initAcc = vectorDef(0.0, 0.0, 0.0);

    Particle *p1 = Particle_Create(
        pos1,
        vel1,
        initAcc,
        2.0,  // Mass
        0.99, // Damping
        0.0   // Start time
    );

    if (!p1)
        return PARTICLE_ERROR_MEMORY;

    // Create second particle
    Vector pos2 = getRandomPosition();
    Vector vel2 = getRandomVelocity();

    Particle *p2 = Particle_Create(
        pos2,
        vel2,
        initAcc,
        2.0,  // Mass
        0.99, // Damping
        0.0   // Start time
    );

    if (!p2)
    {
        Particle_Destroy(p1);
        return PARTICLE_ERROR_MEMORY;
    }

    // Add forces to both particles
    ParticleError error;

    // Add forces to p1
    /* error = Particle_AddForce(p1, Particle_GravityForce, 0.0, INFINITY, NULL);
    if (error != PARTICLE_SUCCESS)
    {
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return error;
    }

    error = Particle_AddForce(p1, Particle_DragForce, 0.0, INFINITY, &dragCoeffs);
    if (error != PARTICLE_SUCCESS)
    {
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return error;
    } */

    // Add forces to p2
    error = Particle_AddForce(p2, Particle_GravityForce, 0.0, INFINITY, NULL);
    if (error != PARTICLE_SUCCESS)
    {
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return error;
    } 

    error = Particle_AddForce(p2, Particle_DragForce, 0.0, INFINITY, &dragCoeffs);
    if (error != PARTICLE_SUCCESS)
    {
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return error;
    } 

    // Create spring parameters for both particles
    SpringParameters *spring1 = malloc(sizeof(SpringParameters));
    SpringParameters *spring2 = malloc(sizeof(SpringParameters));

    if (!spring1 || !spring2)
    {
        free(spring1);
        free(spring2);
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return PARTICLE_ERROR_MEMORY;
    }

    // Set up spring parameters
    spring1->other = p2;
    spring1->springCoeff = SPRING_COEFFICIENT;
    spring1->restLength = SPRING_REST_LENGTH;
    spring1->maxLength = SPRING_MAX_LENGTH;

    spring2->other = p1;
    spring2->springCoeff = SPRING_COEFFICIENT;
    spring2->restLength = SPRING_REST_LENGTH;
    spring2->maxLength = SPRING_MAX_LENGTH;

    // Add spring forces
    /* error = Particle_AddForce(p1, Particle_SpringForce, 0.0, INFINITY, spring1);
    if (error != PARTICLE_SUCCESS)
    {
        free(spring1);
        free(spring2);
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return error;
    } */

    error = Particle_AddForce(p2, Particle_SpringForce, 0.0, INFINITY, spring2);
    if (error != PARTICLE_SUCCESS)
    {
        free(spring1);
        free(spring2);
        Particle_Destroy(p1);
        Particle_Destroy(p2);
        return error;
    }

    // Store particles
    particles[*particleCount].particle = p1;
    particles[*particleCount].active = true;
    particles[*particleCount].pairIndex = *particleCount + 1;

    particles[*particleCount + 1].particle = p2;
    particles[*particleCount + 1].active = true;
    particles[*particleCount + 1].pairIndex = *particleCount;

    *particleCount += 2;
    return PARTICLE_SUCCESS;
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
        "Particle Spring Simulation",
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

    srand((unsigned int)time(NULL));

    ParticleInstance particles[MAX_PARTICLES] = {0};
    int particleCount = 0;
    bool running = true;
    SDL_Event event;

    Uint32 lastTime = SDL_GetTicks();

    while (running)
    {
        while (SDL_PollEvent(&event))
        {
            if (event.type == SDL_QUIT)
            {
                running = false;
            }
            else if (event.type == SDL_KEYDOWN)
            {
                if (event.key.keysym.sym == SDLK_a)
                {
                    ParticleError error = createParticlePair(particles, &particleCount);
                    if (error != PARTICLE_SUCCESS)
                    {
                        fprintf(stderr, "Failed to create particle pair: error %d\n", error);
                    }
                }
            }
        }

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
                Vector pos = particles[i].particle->position;
                if (pos.x < -CIRCLE_RADIUS || pos.x > WINDOW_WIDTH + CIRCLE_RADIUS ||
                    pos.y < -CIRCLE_RADIUS || pos.y > WINDOW_HEIGHT + CIRCLE_RADIUS)
                {
                    // If this particle is part of a pair, deactivate both
                    if (particles[i].pairIndex >= 0)
                    {
                        int pairIdx = particles[i].pairIndex;
                        if (particles[pairIdx].active)
                        {
                            particles[pairIdx].active = false;
                            Particle_Destroy(particles[pairIdx].particle);
                            particles[pairIdx].particle = NULL;
                        }
                    }
                    particles[i].active = false;
                    Particle_Destroy(particles[i].particle);
                    particles[i].particle = NULL;
                }
            }
        }

        // Render
        SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
        SDL_RenderClear(renderer);

        // Draw springs first
        SDL_SetRenderDrawColor(renderer, 100, 100, 255, 255);
        for (int i = 0; i < particleCount; i++)
        {
            if (particles[i].active && particles[i].pairIndex >= 0)
            {
                int pairIdx = particles[i].pairIndex;
                if (particles[pairIdx].active)
                {
                    renderSpring(renderer,
                                 particles[i].particle->position,
                                 particles[pairIdx].particle->position,
                                 WINDOW_HEIGHT);
                }
            }
        }

        // Draw particles
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

    cleanupParticles(particles, particleCount);
    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return 0;
}