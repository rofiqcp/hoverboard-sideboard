#include <math.h>
#include <stdio.h>
#include <string.h>

#include "app_config.h"
#include "eskf_nav.h"
#include "slip_observer.h"

static int failures=0;

static void check(int condition,const char *message)
{
    printf("%s: %s\n",condition?"PASS":"FAIL",message);
    if(!condition)failures++;
}

static void feed(SlipObserver *o,float seconds,float wheel,float predicted,
                 float nis,float gyro,float expected,int expected_valid,
                 float lateral)
{
    const float dt=0.02f;
    int count=(int)(seconds/dt+0.5f);
    for(int i=0;i<count;i++){
        SlipObserverInput in={
            .wheel_speed_mps=wheel,
            .predicted_forward_mps=predicted,
            .wheel_nis=nis,
            .gyro_z_rads=gyro,
            .expected_yaw_rate_rads=expected,
            .expected_yaw_rate_valid=(uint8_t)(expected_valid?1U:0U),
            .lateral_accel_mps2=lateral,
            .dt_s=dt
        };
        slip_observer_update(o,&in);
    }
}

static void test_slip_observer(void)
{
    SlipObserver o;

    slip_observer_init(&o);
    feed(&o,10.0f,2.0f,1.96f,1.0f,0.0f,0.0f,1,0.1f);
    check(slip_observer_get_state(&o)==SLIP_STATE_NORMAL,
          "normal straight driving 10 s stays NORMAL");

    slip_observer_init(&o);
    for(int i=0;i<500;i++){
        const float noise=(float)((i%7)-3)*0.035f;
        SlipObserverInput in={2.0f+noise,2.0f,2.0f,0.0f,0.0f,1U,0.2f,0.02f};
        slip_observer_update(&o,&in);
    }
    check(slip_observer_get_state(&o)==SLIP_STATE_NORMAL,
          "noisy encoder remains NORMAL");

    slip_observer_init(&o);
    for(int i=0;i<500;i++){
        float q=(float)((i%5)-2)*0.045f;
        SlipObserverInput in={q,0.0f,2.5f,0.0f,0.0f,1U,0.0f,0.02f};
        slip_observer_update(&o,&in);
    }
    check(slip_observer_get_state(&o)==SLIP_STATE_NORMAL,
          "near-zero encoder quantization uses wider deadband");

    slip_observer_init(&o);
    feed(&o,2.0f,2.0f,2.0f,1.0f,0.0f,0.0f,1,0.0f);
    {
        SlipObserverInput spike={6.0f,2.0f,80.0f,2.0f,0.0f,1U,5.0f,0.02f};
        slip_observer_update(&o,&spike);
    }
    check(slip_observer_get_state(&o)!=SLIP_STATE_SEVERE,
          "single spike never escalates directly to SEVERE");

    slip_observer_init(&o);
    feed(&o,4.0f,5.0f,2.0f,25.0f,0.0f,0.0f,0,0.3f);
    check(slip_observer_get_state(&o)==SLIP_STATE_SEVERE ||
          slip_observer_get_state(&o)==SLIP_STATE_SLIP,
          "persistent wheel spin reaches SLIP/SEVERE");

    slip_observer_init(&o);
    feed(&o,3.5f,0.2f,3.0f,22.0f,0.0f,0.0f,0,0.4f);
    check(slip_observer_get_state(&o)==SLIP_STATE_SLIP ||
          slip_observer_get_state(&o)==SLIP_STATE_SEVERE,
          "persistent braking/skid pattern is detected");

    slip_observer_init(&o);
    feed(&o,10.0f,2.05f,2.0f,1.5f,0.60f,0.58f,1,2.2f);
    check(slip_observer_get_state(&o)==SLIP_STATE_NORMAL,
          "normal turn with matching expected yaw rate does not false-trigger");

    slip_observer_init(&o);
    feed(&o,3.0f,2.0f,2.0f,6.0f,1.2f,0.2f,1,0.5f);
    check(slip_observer_get_score(&o)>0.12f,
          "persistent yaw mismatch raises slip score");

    slip_observer_init(&o);
    feed(&o,4.0f,5.0f,2.0f,25.0f,0.0f,0.0f,0,3.0f);
    check(slip_observer_get_state(&o)==SLIP_STATE_SEVERE,
          "strong persistent wheel spin reaches SEVERE before recovery");
    feed(&o,0.50f,2.0f,2.0f,1.0f,0.0f,0.0f,1,0.1f);
    check(slip_observer_get_state(&o)!=SLIP_STATE_NORMAL,
          "recovery is delayed rather than immediate");
    feed(&o,7.0f,2.0f,2.0f,1.0f,0.0f,0.0f,1,0.1f);
    check(slip_observer_get_state(&o)==SLIP_STATE_NORMAL,
          "consistent measurements return observer to NORMAL");

    {
        SlipObserver n,s,l,v;
        slip_observer_init(&n); slip_observer_init(&s);
        slip_observer_init(&l); slip_observer_init(&v);
        n.state=SLIP_STATE_NORMAL;
        s.state=SLIP_STATE_SUSPECT;
        l.state=SLIP_STATE_SLIP; l.lateral_evidence=1.0f;
        v.state=SLIP_STATE_SEVERE; v.lateral_evidence=1.0f;
        check(slip_observer_wheel_sigma_scale(&n)<=slip_observer_wheel_sigma_scale(&s) &&
              slip_observer_wheel_sigma_scale(&s)<=slip_observer_wheel_sigma_scale(&l) &&
              slip_observer_wheel_sigma_scale(&l)<=slip_observer_wheel_sigma_scale(&v),
              "wheel sigma scale is monotonic with severity");
        check(slip_observer_nhc_sigma_scale(&n)<=slip_observer_nhc_sigma_scale(&s) &&
              slip_observer_nhc_sigma_scale(&s)<=slip_observer_nhc_sigma_scale(&l) &&
              slip_observer_nhc_sigma_scale(&l)<=slip_observer_nhc_sigma_scale(&v),
              "NHC sigma scale is monotonic with severity");
        check(!slip_observer_reject_wheel(&l) && slip_observer_reject_wheel(&v),
              "SEVERE rejects wheel while SLIP only deweights it");
    }
}

static void test_eskf_agent2_api(void)
{
    const float g=GRAVITY_MPS2;
    const float a0[3]={0.0f,0.0f,g};
    EskfNav f;
    eskf_nav_init(&f,a0);

    check(eskf_nav_covariance_psd_check(&f),
          "covariance PSD check accepts healthy initialized P");

    {
        EskfNav moving=f;
        for(int k=1;k<=12000;k++){
            float t=k*0.01f;
            float w[3]={0.05f*sinf(0.2f*t),0.04f*cosf(0.17f*t),0.35f*sinf(0.11f*t)};
            float ac[3]={0.8f*sinf(0.13f*t),0.4f*cosf(0.19f*t),g};
            float da[3],dv[3];
            for(int i=0;i<3;i++){da[i]=w[i]*0.01f;dv[i]=ac[i]*0.01f;}
            if(!eskf_nav_predict_delta(&moving,da,dv,0.01f))break;
        }
        check(eskf_nav_covariance_psd_check(&moving),
              "covariance PSD check accepts 120 s healthy propagated P");
    }

    {
        EskfNav invalid=f;
        invalid.P[0][1]=invalid.P[1][0]=1.0f;
        check(!eskf_nav_covariance_psd_check(&invalid),
              "covariance PSD check rejects indefinite synthetic P");
    }

    {
        float world_v[3]={0.0f,1.0f,0.0f};
        float body_v[3]={0.0f,0.0f,0.0f};
        check(eskf_nav_reset_world_velocity(&f,world_v,0.10f),
              "body velocity test sets world velocity");
        check(eskf_nav_reset_yaw(&f,1.57079632679f,0.05f),
              "body velocity test sets +90 deg yaw");
        check(eskf_nav_get_body_velocity(&f,body_v),
              "body velocity API succeeds");
        check(fabsf(body_v[0]-1.0f)<1e-4f &&
              fabsf(body_v[1])<1e-4f && fabsf(body_v[2])<1e-4f,
              "body convention is +X forward, +Y left, +Z up");
    }

    eskf_nav_init(&f,a0);
    {
        float wheel[3]={0.0f,0.0f,0.0f};
        check(eskf_nav_fuse_body_velocity(&f,wheel,0x01U,0.10f),
              "wheel aiding accepts consistent forward velocity");
        EskfNavDiagnostics d;
        eskf_nav_get_diagnostics(&f,&d);
        check(d.wheel_accept_count==1U && d.wheel_reject_count==0U &&
              d.last_wheel_reason==ESKF_DIAG_REASON_NONE &&
              isfinite(d.last_wheel_nis),
              "wheel diagnostics expose accepted innovation/NIS");
    }

    {
        EskfNav nis_filter;
        eskf_nav_init(&nis_filter,a0);
        float wheel_nis_bad[3]={1.0f,0.0f,0.0f};
        check(!eskf_nav_fuse_body_velocity(&nis_filter,wheel_nis_bad,0x01U,0.01f),
              "wheel aiding can reject by NIS below hard innovation limit");
        EskfNavDiagnostics d;
        eskf_nav_get_diagnostics(&nis_filter,&d);
        check(d.last_wheel_reason==ESKF_DIAG_REASON_NIS_GATE &&
              d.last_wheel_nis>9.0f,
              "wheel diagnostics distinguish NIS gate rejection");
    }

    {
        float wheel_bad[3]={4.0f,0.0f,0.0f};
        check(!eskf_nav_fuse_body_velocity(&f,wheel_bad,0x01U,0.10f),
              "wheel aiding rejects innovation beyond hard limit");
        EskfNavDiagnostics d;
        eskf_nav_get_diagnostics(&f,&d);
        check(d.wheel_reject_count==1U &&
              d.last_wheel_reason==ESKF_DIAG_REASON_INNOVATION_LIMIT &&
              isfinite(d.last_wheel_nis),
              "wheel diagnostics explain hard innovation rejection with NIS");
    }

    {
        check(!eskf_nav_fuse_yaw(&f,2.0f,0.05f),
              "yaw aiding rejects hard innovation limit");
        EskfNavDiagnostics d;
        eskf_nav_get_diagnostics(&f,&d);
        check(d.yaw_reject_count==1U &&
              d.last_yaw_reason==ESKF_DIAG_REASON_INNOVATION_LIMIT &&
              fabsf(d.last_yaw_innovation_rad)>1.0f &&
              isfinite(d.last_yaw_nis),
              "yaw diagnostics expose wrapped innovation/NIS and reject reason");
        check(d.covariance_psd_ok==1U,
              "diagnostics getter reports current covariance PSD health");
    }

    {
        float vel[3]={0.1f,-0.1f,0.0f};
        float pos[3]={0.2f,0.1f,0.0f};
        check(eskf_nav_fuse_world_velocity(&f,vel,0.5f),
              "world velocity aiding accepted for diagnostics");
        check(eskf_nav_fuse_world_position(&f,pos,1.0f),
              "world position aiding accepted for diagnostics");
        EskfNavDiagnostics d;
        eskf_nav_get_diagnostics(&f,&d);
        check(d.velocity_accept_count==1U &&
              d.last_velocity_innovation_norm_mps>0.0f &&
              isfinite(d.last_velocity_nis_max),
              "world velocity diagnostics expose norm and max NIS");
        check(d.position_accept_count==1U &&
              d.last_position_innovation_norm_m>0.0f &&
              isfinite(d.last_position_nis_max),
              "world position diagnostics expose norm and max NIS");
    }
}

int main(void)
{
    test_slip_observer();
    test_eskf_agent2_api();
    return failures?1:0;
}
