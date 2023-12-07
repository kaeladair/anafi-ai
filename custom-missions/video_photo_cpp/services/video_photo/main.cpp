#include <csignal>
#include <libpomp.hpp>
#include <unistd.h>

#include "mission_controller.hpp"

#define ULOG_TAG video_photo
#include <ulog.h>
ULOG_DECLARE_TAG(ULOG_TAG);

/************************************************************************************/

class Context {
	/* Main loop of the program */
	pomp::Loop mLoop;

	/* Mission controller interface, responsible for commands and events
	 * handling
	 */
	MissionController missionController;

public:
	Context() : missionController(this->mLoop) {}

	inline void wakeup()
	{
		this->mLoop.wakeup();
	}
	inline void waitAndProcess(int timeout)
	{
		this->mLoop.waitAndProcess(timeout);
	}
	inline int start()
	{
		return this->missionController.start();
	}
};

/*
 * Global context of the mission. It retrieves all the objects and
 * controllers the mission needs to work up
 */
static Context s_ctx;

/************************************************************************************/

/* Stop flag, set to 1 by signal handler to exit cleanly */
static sig_atomic_t stop;

static void sighandler(int signum)
{
	/* Set stopped flag and wakeup loop */
	ULOGI("Signal %d (%s) received", signum, strsignal(signum));
	stop = 1;
	s_ctx.wakeup();
}

/************************************************************************************/

int main(int argc, char *argv[])
{
	int res = 0;

	/* Initialisation code
	 *
	 * The service is automatically started by the drone when the mission is
	 * loaded.
	 */
	ULOGI("Hello from video_photo mission");
	/* Setup signal handler */
	signal(SIGINT, &sighandler);
	signal(SIGTERM, &sighandler);
	signal(SIGPIPE, SIG_IGN);

	/* Initialize and start context */
	res = s_ctx.start();
	if (res != 0) {
		ULOGE("Error while starting MissionController");
		ULOG_ERRNO("MissionController::start", -res);
		return res;
	} else
		ULOGI("MissionController has started successfully");

	/* Loop code
	 *
	 * The service is assumed to run an infinite loop, and termination
	 * requests are handled via a SIGTERM signal.
	 * If your serivce exists before this SIGTERM is sent, it will be
	 * considered as a crash, and the system will relaunch the service.
	 * If this happens too many times, the system will no longer start the
	 * service.
	 */
	/* Run loop until stop is requested */
	while (!stop)
		s_ctx.waitAndProcess(-1);

	/* Cleanup code
	 *
	 * When stopped by a SIGTERM, a service can use a short amount of time
	 * for cleanup (typically closing opened files and ensuring that the
	 * written data is coherent).
	 */
	ULOGI("End of video_photo mission");

	return res == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}