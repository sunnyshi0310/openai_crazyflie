# openai_crazyflie

This project describes how to implement openai in the swarm of the Crazyflie robots system. 
It is built upon the ``openai_ros`` and ``CrazyS`` repositories.

    # My Task Env: Crazyflie
    elif task_env == 'Crazyflie-v0':

        register(
            id=task_env,
            entry_point='openai_ros.task_envs.Crazyflie.cf_goto:CrazyflieGotoEnv',
            max_episode_steps=max_episode_steps,
        )

        # import our training environment
        from openai_ros.task_envs.Crazyflie import cf_goto

