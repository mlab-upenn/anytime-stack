#include <iostream>
#include <sys/ipc.h>
#include <sys/shm.h>

using namespace std;

int main()
{
    // ftok to generate unique key
    key_t key = ftok("shmfile", 65);
    
    // shmget returns an identifier in shmid
    int shmid = shmget(key, 1024, 0666 | IPC_CREAT);
  
    // shmat to attach to shared memory
    int *data = (int*) shmat(shmid, (void*)0, 0);

    cout << "Read data: " << *data << endl;

    shmdt(data);

    shmctl(shmid, IPC_RMID, NULL);

    return 0;
}