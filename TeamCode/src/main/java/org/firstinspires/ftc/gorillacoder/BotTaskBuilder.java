package org.firstinspires.ftc.gorillacoder;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

public class BotTaskBuilder<OpModeT extends OpMode> {

    public static class Exception extends java.lang.Exception {
        public Exception(String format, Object... args) {
            super(String.format(format, args));
        }

        public Exception(Exception cause, String format, Object... args) {
            super(String.format(format, args), cause);
        }
    }

    public BotTaskBuilder<OpModeT> opMode(OpModeT value) {
        myOpMode = value;
        return this;
    }

    public BotTaskBuilder<OpModeT> addTask(BotTask<OpModeT> task) {
        assert null == task.opMode();
        task.opMode(myOpMode);
        myTasks.add(task);
        return this;
    }

    @SafeVarargs
    public final BotTaskBuilder<OpModeT> addTasks(BotTask<OpModeT>... tasks) {
        for (BotTask<OpModeT> task: tasks) {
            addTask(task);
        }
        return this;
    }

    // change throws to BotTaskBuilder.Exception
    public BotTaskBuilder<OpModeT> addTask(Class< ? extends BotTask<OpModeT>> clazz)
            throws IllegalAccessException, InstantiationException
    {
        BotTask<OpModeT> task = clazz.newInstance();
        addTask(task);
        return this;
    }

    @SafeVarargs
    public final BotTaskBuilder<OpModeT> addTasks(Class< ? extends BotTask<OpModeT>>... tasks)
            throws IllegalAccessException, InstantiationException
    {
        for (Class< ? extends BotTask<OpModeT>> task: tasks) {
            addTask(task);
        }
        return this;
    }

    // change throws to BotTaskBuilder.Exception
    public BotTaskBuilder<OpModeT> addTask(String taskName)
            throws ClassNotFoundException, IllegalAccessException, InstantiationException
    {
        @SuppressWarnings("unchecked")
        Class<? extends BotTask<OpModeT>> task =
                (Class<? extends BotTask<OpModeT>>) Class.forName(taskName).asSubclass(BotTask.class);
        addTask(task);
        return this;
    }

    // change throws to BotTaskBuilder.Exception
    @SuppressWarnings("unchecked")
    public BotTaskBuilder<OpModeT> addTask(Object task)
            throws BotTaskBuilder.Exception, ClassNotFoundException, IllegalAccessException, InstantiationException
    {
        if (task instanceof BotTask) {
            addTask((BotTask<OpModeT>) task);
            return this;
        }
        if (task instanceof Class) {
            Class<? extends BotTask<OpModeT>> clazz = (Class<? extends BotTask<OpModeT>>) ((Class<?>)task).asSubclass(BotTask.class);
            addTask(clazz);
            return this;
        }
        if (task instanceof String) {
            addTask((String)task);
            return this;
        }

        throw new Exception("Can't add %s to task list. task must be a BotTask instance, a BotTask subclass, or the name of a BotTask subclass");
    }
    public BotTaskBuilder<OpModeT> addTasks(Object... tasks)
            throws BotTaskBuilder.Exception, ClassNotFoundException, IllegalAccessException, InstantiationException {
        for (Object task: tasks) {
            addTask(task);
        }
        return this;
    }

    public List<BotTask<OpModeT>> tasks() {
        return Collections.unmodifiableList(myTasks);
    }

    private OpModeT myOpMode;

    List<BotTask<OpModeT>> myTasks = new ArrayList<>(100);
} // class BotTaskBuilder<OpModeT>
