class AP_Swarm
{
public:
    AP_Swarm();

    /* Do not allow copies */
    AP_Swarm(const AP_Swarm &other) = delete;
    AP_Swarm &operator=(const AP_Swarm&) = delete;

    // get singleton instance
    static AP_Swarm *instance(void) {
        return _instance;
    }

    // initialisation
    void init(void);

    /// update - allow updates of leds that cannot be updated during a timed interrupt
    void update(void);

private:

    static AP_Swarm *_instance;
};
