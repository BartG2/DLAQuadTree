#include <chrono>
#include <random>
#include "raylib.h"
#include <iostream>
#include <vector>
#include <future>
#include <omp.h>
#include <array>
#include <algorithm>
#include <unordered_set>
#include <thread>
#include <functional>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <memory>
#include <list>

std::mt19937 CreateGeneratorWithTimeSeed();
float RandomFloat(float min, float max, std::mt19937& rng);
bool Contains(const Rectangle& r1, const Rectangle& r2);

//-------------------------------------------------------------------------------------------------------------------------------------------------------

constexpr int screenWidth = 2560, screenHeight = 1440, numThreads = 2, maxTreeDepth = 5;
const float collisionThreshold = 3.0f;

const Vector2 particleSize = {2,2};

std::mt19937 rng = CreateGeneratorWithTimeSeed();

//-------------------------------------------------------------------------------------------------------------------------------------------------------

class Particle {
public:
    Vector2 pos;
    Color color;
    bool isStuck;

    Particle(float x, float y, Color col)
        :pos({x,y}), color(col), isStuck(false)
    {}

    Particle()
        :pos({ screenWidth - 10, screenHeight - 10 }), color(WHITE), isStuck(false)
    {}

    void RandomWalk(float stepSize, int numSteps) {
        if(!isStuck){
            for (int i = 0; i < numSteps; i++) {
                float dx = RandomFloat(-1, 1, rng);
                float dy = RandomFloat(-1, 1, rng);

                float newX = pos.x + dx * stepSize;
                float newY = pos.y + dy * stepSize;

                // Check if particle is out of bounds and correct position
                if (newX < 0) {
                    newX = 0;
                }
                else if (newX > screenWidth) {
                    newX = screenWidth;
                }
                if (newY < 0) {
                    newY = 0;
                }
                else if (newY > screenHeight) {
                    newY = screenHeight;
                }

                pos.x = newX;
                pos.y = newY;
            }
        }
    }

};

class Timer{
public:
    std::chrono::time_point<std::chrono::high_resolution_clock> startPoint;

    Timer(){
        startPoint = std::chrono::high_resolution_clock::now();
    }

    ~Timer(){
        stop();
    }

    void stop(){
        auto endPoint = std::chrono::high_resolution_clock::now();
        auto start = std::chrono::time_point_cast<std::chrono::microseconds>(startPoint).time_since_epoch().count();
        auto end = std::chrono::time_point_cast<std::chrono::microseconds>(endPoint).time_since_epoch().count();

        auto duration = end - start;
        double ms = duration * 0.001;

        std::cout << "ms: " << ms << std::endl;
    }
};

class QuadTree{
public:

    int currentDepth;
    Rectangle currentSize;
    std::vector<Particle> particles;
    std::array<std::shared_ptr<QuadTree>, 4> children{};

    QuadTree(const int setDepth, const Rectangle& setSize){
        currentDepth = setDepth;
        currentSize = setSize;
    }

    void clear(){
        particles.clear();

        for(int i = 0; i < 4; i++){
            if(children[i]){
                children[i]->clear();
            }
            children[i].reset();
        }
    }

    void insert(const Particle& newParticle){
        for(int i = 0 ; i < 4; i++){
            if(children[i]){
                if(CheckCollisionPointRec(newParticle.pos, children[i]->currentSize)){
                    if(currentDepth + 1 < maxTreeDepth){
                        children[i]->insert(newParticle);
                        return;
                    }
                }
            }
        }

        //didn't fit in children, so must go here
        particles.push_back(newParticle);
    }

    std::list<Particle> search(Vector2 center, float radius){
        std::list<Particle> result;

        // Check if the search area intersects the QuadTree node's boundary
        if(!CheckCollisionCircleRec(center, radius, currentSize)) {
            return result;
        }

        // If this node has particles, add the ones within the search area to the result list
        for(const auto& particle : particles){
            if(CheckCollisionPointCircle(particle.pos, center, radius)){
                result.push_back(particle);
            }
        }

        // Recursively search the children nodes
        for(int i = 0; i < 4; i++){
            if(children[i]){
                auto childResult = children[i]->search(center, radius);
                result.splice(result.end(), childResult);
            }
        }

        return result;
    }

    int size() const{
        int count = particles.size();

        for(int i = 0 ; i < 4; i++){
            if(children[i]){
                count += children[i]->size();
            }
        }

        return count;
    }

    void draw() const{
        for(const auto& particle : particles){
            DrawPixelV(particle.pos, RED);
        }

        for(int i = 0; i < 4; i++){
            if(children[i]){
                children[i]->draw();
            }
        }
    }

};

//-------------------------------------------------------------------------------------------------------------------------------------------------------

std::vector<Particle> freeParticles, aggregateParticles;

//-------------------------------------------------------------------------------------------------------------------------------------------------------

std::mt19937 CreateGeneratorWithTimeSeed() {
    // Get the current time in nanoseconds
    auto now = std::chrono::high_resolution_clock::now();
    auto nanos = std::chrono::time_point_cast<std::chrono::nanoseconds>(now).time_since_epoch().count();

    // Create a new mt19937 generator and seed it with the current time in nanoseconds
    std::mt19937 gen(static_cast<unsigned int>(nanos));
    return gen;
}

float RandomFloat(float min, float max, std::mt19937& rng) {
    std::uniform_real_distribution<float> dist(min, max);
    return dist(rng);
}

float vector2distance(Vector2 v1, Vector2 v2) {
    float dx = v2.x - v1.x;
    float dy = v2.y - v1.y;
    return std::sqrt(dx * dx + dy * dy);
}

std::vector<Particle> CreateCircle(int numParticles, Color col, Vector2 center, float radius){
    float degreeIncrement = 360.0f/(float)numParticles;
    std::vector<Particle> particles;

    for(float i = 0; i < 360; i += degreeIncrement){
        float x = radius * cos(i) + center.x;
        float y = radius * sin(i) + center.y;
        Particle p(x,y,col);
        particles.push_back(p);
    }

    return particles;
}

bool vectorsEqual(Vector2 v1, Vector2 v2){
    if(v1.x == v2.x && v1.y == v2.y){
        return true;
    }
    else{
        return false;
    }
}

void primitiveCollisionCheck(){
    for(unsigned int i = 0; i < aggregateParticles.size(); i++){
        for(unsigned int j = 0; j < freeParticles.size(); j++){
            if(CheckCollisionPointCircle(freeParticles[j].pos, aggregateParticles[i].pos, collisionThreshold)){
                freeParticles[j].color = WHITE;
                aggregateParticles.push_back(freeParticles[j]);
                freeParticles.erase(freeParticles.begin() + j);
            }
        }
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------

void Initialize(){
    InitWindow(screenWidth, screenHeight, "DLA, hopefully");
    SetTargetFPS(100);

    constexpr int startingNumParticles = 100, startingRadius = 100;
    const Color startingColor = RED;
    const Vector2 startingCenter = {screenWidth / 2, screenHeight / 2};

    freeParticles = CreateCircle(startingNumParticles, startingColor, startingCenter, startingRadius);
    aggregateParticles = {1, Particle(screenWidth / 2.0, screenHeight / 2.0, WHITE)};
}

void DrawParticlesVector(const std::vector<Particle>& particles){
    for(unsigned int i = 0; i < particles.size(); i++){
        DrawPixelV(particles[i].pos, particles[i].color);
    }
}

void RandomWalkAll(std::vector<Particle>& particles){
    
    for(auto& p : particles){
        p.RandomWalk(1, 1);
    }
}

void ConcentricCircles(int frameCount){
    if(frameCount / 5 < screenHeight / 2 && frameCount % 500 == 0){
        std::vector<Particle> fp2 = CreateCircle(40 * (1 + frameCount / 50),RED,{screenWidth/2.0, screenHeight/2.0}, 60 + frameCount / 5);
        freeParticles.insert(freeParticles.end(), fp2.begin(), fp2.end());
    }
}

QuadTree initializeQT(){
    QuadTree qt(0, Rectangle{0, 0, screenWidth, screenHeight});

    qt.insert(freeParticles[0]);

    return qt;
}

//-------------------------------------------------------------------------------------------------------------------------------------------------------

int main(){
    Initialize();
    
    for(int frameCount = 0; !WindowShouldClose(); frameCount++){

        ConcentricCircles(frameCount);
        RandomWalkAll(freeParticles);

        QuadTree qt = initializeQT();
        
        BeginDrawing();
        {
            ClearBackground(BLACK);
            DrawFPS(10,10);

            DrawParticlesVector(aggregateParticles);

            qt.draw();

        }
        EndDrawing();

    }

    return 0;
}
