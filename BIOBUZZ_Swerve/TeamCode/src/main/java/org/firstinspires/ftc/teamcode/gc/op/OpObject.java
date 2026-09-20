package org.firstinspires.ftc.teamcode.gc.op;

import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.IOException;
import java.util.Enumeration;
import java.util.HashSet;

import dalvik.system.DexFile;

public abstract class OpObject extends OpMode {
    /*-------------------------------------------------------------
     * Static logic
     * -------------------------------------------------------------*/
    private final static HashSet<OpBehavior> s_globalBehaviors = new HashSet<>();
    private static OpObject s_current = null;
    public static void RegisterGlobalBehavior(OpBehavior behavior){
        s_globalBehaviors.add(behavior);
    }

    static {
        // TODO Reflection Utils
//        try{
//            Context context = AppUtil.getDefContext();
//            DexFile df = new DexFile(context.getPackageCodePath());
//            for(Enumeration<String> iter = df.entries(); iter.hasMoreElements();){
//                String className = iter.nextElement();
//                if(!className.startsWith("org.firstinspires.ftc.teamcode"))
//                    continue;
//
//                try{
//                    Class<?> clazz = Class.forName(className, true, context.getClassLoader());
//                } catch (Exception e) {
//
//                }
//            }
//        } catch (IOException e){
//
//        }
    }

    /*-------------------------------------------------------------
    * Instance logic
    * -------------------------------------------------------------*/
    private final HashSet<OpBehavior> m_behaviors = new HashSet<>();

    public abstract void Init();
    public abstract void Start();
    public abstract void Loop();
    public abstract void Stop();

    @Override
    public final void init() {
        s_current = this;
        Init();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Init();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            telemetry.addLine("init sub-system " + globalBehavior.getClass().getSimpleName());
            globalBehavior.m_object = this;
            globalBehavior.Init();
        }
    }

    @Override
    public void start() {
        Start();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Start();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.Start();
        }
    }

    @Override
    public final void loop() {
        Loop();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Loop();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.Loop();
        }
    }

    @Override
    public final void stop() {
        Stop();
        for (OpBehavior behavior : m_behaviors) {
            behavior.Stop();
        }
        for (OpBehavior globalBehavior : s_globalBehaviors) {
            globalBehavior.Stop();
            globalBehavior.m_object = null;
        }
        s_current = null;
    }

    public final void RegisterBehavior(OpBehavior behavior) {
        behavior.m_object = this;
        m_behaviors.add(behavior);
    }

    public final void UnregisterBehavior(OpBehavior behavior) {
        if(behavior.m_object == this) {
            m_behaviors.remove(behavior);
            behavior.m_object = null;
        }
    }
}
