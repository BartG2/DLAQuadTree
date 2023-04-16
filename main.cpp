#include <chrono>
#include <random>
#include "raylib.h"
#include <iostream>
#include <vector>
#include <array>
#include <fstream>
#include <cmath>
#include <memory>
#include <list>

//---------------------------------------------------------------------------------------------------------------------------------

std::mt19937 CreateGeneratorWithTimeSeed();
float RandomFloat(float min, float max, std::mt19937& rng);
int RandomInt(int min, int max, std::mt19937& rng);

//---------------------------------------------------------------------------------------------------------------------------------


const int screenWidth = 2440, screenHeight = 1368, maxTreeDepth = 5;

std::mt19937 rng = CreateGeneratorWithTimeSeed();

//---------------------------------------------------------------------------------------------------------------------------------

class Particle{
public:
    Vector2 pos;
    Color color;

    Particle(Vector2 position, Color col){
        pos = position;
        color = col;
    }
};

enum CreatureType{
    GenericPrey,
    GenericPredator
};

class Creature{
public:
    CreatureType species;
    double energy;
    int initialEnergy;
    double maxSpeed;
    int initialMaxSpeed;
    float direction;
    Vector2 position;
    Vector2 velocity;
    float sightRange;
    double size;
    double health;
    double initialHealth;
    double energyCost;
    int foodLevel;
    int waterLevel;
    unsigned long age;
    bool alive;

    Creature(CreatureType type, int speed, float range){
        species = type;
        maxSpeed = speed;
        initialMaxSpeed = speed;
        sightRange = range;
        position = {RandomFloat(0, screenWidth, rng), RandomFloat(0, screenHeight, rng)};
        direction = RandomFloat(0, 360, rng);
        health = 100;
        initialHealth = 100;
        size = 1;
        energy = 100000;
        initialEnergy = 100000;
        alive = true;
        age = 0;
    }

    double calculateEnergyCost(double maxSpeed, int sightRange, int size){
        static constexpr double sightCost = 1;
        return 100*size*size*size + maxSpeed*maxSpeed + sightCost*sightRange;
    }

    void move(){
        float newX = position.x + maxSpeed*cos(direction);
        float newY = position.y + maxSpeed*sin(direction);
        if(newX <= screenWidth and newX >= 0 and newY >= 0 and newY <= screenHeight){
            position = {newX, newY};
        }
        else{
            direction -= 180;
        }
    }

    void incrementalRandomWalk(float dv){
        float dx, dy;  

        do
        {
            float dx = RandomFloat(-dv, dv, rng);
            float dy = RandomFloat(-dv, dv, rng);
        } while ((position.x > screenWidth and dx > 0) or (position.x < 0 and dx < 0) or (position.y > screenHeight and dy > 0) or (position.y < screenHeight and dy < 0));
        
        velocity = {dx, dy};
    }

    void shiftDirectionRandomly(float magnitude){
        float d = RandomFloat(-magnitude, magnitude, rng);

        direction += d;
        if(direction > 360){
            direction -=  360;
        }
        else if(direction < 0){
            direction += 360;
        }


    }

    void update(){
        age++;

        /*double ageSpeedDecayFactor = 1 / 1000000.0;
        double speedPeakAge = 500;
        double ds = double(initialMaxSpeed) - ageSpeedDecayFactor*(age-speedPeakAge)*(age-speedPeakAge);
        if(ds >= 0){
            maxSpeed = ds;
        }
        std::cout << age << ", " << maxSpeed << std::endl;*/

        double energyCost = calculateEnergyCost(maxSpeed, sightRange, size);
        if(energy - energyCost > 0){
            energy -= energyCost;
        }
        else{
            die();
        }

        move();
        shiftDirectionRandomly(0.4);
    }

    void die(){
        alive = false;
    }
};

class QuadTree{
public:

    int currentDepth;
    Rectangle currentSize;
    std::vector<Particle> particles;
    std::array<std::shared_ptr<QuadTree>, 4> children{};
    std::array<Rectangle, 4> childAreas{};

    QuadTree(const int setDepth, const Rectangle& setSize){
        currentDepth = setDepth;
        resize(setSize);
    }

    void resize(const Rectangle& setSize){
        clear(); 
        currentSize = setSize;

        float newWidth = currentSize.width / 2.0f, newHeight = currentSize.height / 2.0f;
        float x = currentSize.x, y = currentSize.y;

        childAreas = {
            Rectangle{x + newWidth, y, newWidth, newHeight},
            Rectangle{x, y, newWidth, newHeight},
            Rectangle{x, y + newHeight, newWidth, newHeight},
            Rectangle{x + newWidth, y + newHeight, newWidth, newHeight}
        };

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
            if(CheckCollisionPointRec(newParticle.pos, childAreas[i])){
                if(currentDepth + 1 < maxTreeDepth){
                    if(!children[i]){
                        children[i] = std::make_shared<QuadTree>(currentDepth + 1, childAreas[i]);
                    }
                    children[i]->insert(newParticle);
                    return;
                }
            }
        }

        //didn't fit in children, so must go here
        particles.emplace_back(newParticle);
    }

    std::list<Particle> search(Vector2& center, float radius, bool removeSearched){
        std::list<Particle> result;

        // Check if the search area intersects the QuadTree node's boundary
        if(!CheckCollisionCircleRec(center, radius, currentSize)) {
            return result;
        }

        // If this node has particles, add the ones within the search area to the result list
        for(unsigned int i = 0; i < particles.size(); i++){
            if(CheckCollisionPointCircle(particles[i].pos, center, radius)){
                result.push_back(particles[i]);
                if(removeSearched){
                    particles.erase(particles.begin() + i);
                }
            }
        }

        // Recursively search the children nodes
        for(int i = 0; i < 4; i++){
            if(children[i]){
                auto childResult = children[i]->search(center, radius, removeSearched);
                result.splice(result.end(), childResult);
            }
        }

        return result;
    }

    std::vector<Particle> returnAll(int depth){
        std::vector<Particle> result;

        if(currentDepth >= depth){
            result.insert(result.end(), particles.begin(), particles.end());
        }

        for(int i = 0; i < 4; i++){
            if(children[i]){
                auto childResult = children[i]->returnAll(depth);
                result.insert(result.end(), childResult.begin(), childResult.end());
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
            DrawPixelV(particle.pos, particle.color);
        }

        //DrawRectangleLinesEx(currentSize, 0.7, GREEN);

        for(int i = 0; i < 4; i++){
            if(children[i]){
                children[i]->draw();
            }
        }
    }

};

//---------------------------------------------------------------------------------------------------------------------------------

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

int RandomInt(int min, int max, std::mt19937& rng){
    std::uniform_int_distribution<int> dist(min, max);
    return dist(rng);
}


//---------------------------------------------------------------------------------------------------------------------------------

void initialize(){
    InitWindow(screenWidth, screenHeight, "Predator and Prey Sim");
    SetTargetFPS(100);
}

void drawBackground(){
    ClearBackground(RAYWHITE);
    DrawFPS(screenWidth - 40, 20);
}

void run(){
    initialize();

    std::ofstream outFile;
    outFile.open("data.csv");
    outFile.close();

    for(int frame = 0; !WindowShouldClose(); frame++) {

        BeginDrawing();

        drawBackground();

        EndDrawing();
    }

    CloseWindow();
}

void drawHealthBar(Creature& creature, int barWidth, int barHeight, int verticalOffset, Color fullColor, Color emptyColor, int barType){
    double interbarDistance = 8;

    //energy bar
    if(barType == 1){
        DrawRectangle(creature.position.x - barWidth / 2, creature.position.y - verticalOffset, barWidth*(creature.energy/creature.initialEnergy), barHeight, fullColor);
        DrawRectangle(creature.position.x - barWidth / 2 + barWidth*(creature.energy/creature.initialEnergy), creature.position.y - verticalOffset, barWidth*(creature.initialEnergy - creature.energy) / creature.initialEnergy, barHeight,  emptyColor);
    }
    //health bar
    else if(barType == 2){
        DrawRectangle(creature.position.x - barWidth / 2, creature.position.y - verticalOffset - interbarDistance, barWidth*(creature.health/creature.initialHealth), barHeight, fullColor);
        DrawRectangle(creature.position.x - barWidth / 2 + barWidth*(creature.health/creature.initialHealth), creature.position.y - verticalOffset - interbarDistance, barWidth*(creature.initialHealth - creature.health) / creature.initialHealth, barHeight,  emptyColor);
    }
}

std::vector<Creature> initializeRandomCreatures(int number, CreatureType type){
    std::vector<Creature> creatures(number, {type, 1, 2});
    for(int i = 0; i < number; i++){

    }
}

//---------------------------------------------------------------------------------------------------------------------------------

int main() {
    initialize();

    Creature adam(GenericPredator, 1, 1);

    int numPredators = 10, numPrey = 20;

    std::vector<Creature> predators(numPredators, {GenericPredator, 1, 2});;
    std::vector<Creature> prey(numPrey, {GenericPrey, 2, 1});
    std::vector<Particle> predatorLocations;
    std::vector<Particle> preyLocations;
    std::cout << predators.size() << std::endl;

    for(int frame = 0; !WindowShouldClose(); frame++){

        BeginDrawing();

        drawBackground();

        for(int i = 0 ; i < numPredators; i++){
            drawHealthBar(predators[i], 40, 3, 20, GREEN, RED, 1);
            drawHealthBar(predators[i], 40, 3, 20, BLUE, ORANGE, 2);
            DrawCircleV(predators[i].position, 10, BLUE);
        }

        for(int i = 0; i < numPrey; i++){
            drawHealthBar(prey[i], 40, 3, 20, GREEN, RED, 1);
            drawHealthBar(prey[i], 40, 3, 20, BLUE, ORANGE, 2);
            DrawCircleV(prey[i].position, 10, GREEN);
        }

        /*if(adam.alive){
            drawHealthBar(adam, 40, 3, 20, GREEN, RED, 1);
            drawHealthBar(adam, 40, 3, 20, BLUE, ORANGE, 2);
            DrawCircleV(adam.position, 10, GREEN);
            adam.update();
        }
        else{
            //std::cout << "dead" << std::endl;
            DrawCircleV(adam.position, 10, RED);
        }*/

        EndDrawing();

        for(int i = 0; i < numPredators; i++){
            predators[i].update();
        }
        for(int i = 0; i < numPrey; i++){
            prey[i].update();
        }

    }

    return 0;
}
