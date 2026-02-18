use core::cell::Cell;
use rs_matter::{
    dm::{Access, Cluster, Dataver, HandlerContext, Quality},
    error::Error,
};

use crate::clusters::decl::identify::{
    self, AttributeId, CommandId, IdentifyRequest, IdentifyTypeEnum, TriggerEffectRequest,
};

use super::decl::identify::ClusterAsyncHandler;

pub const CLUSTER_ID: u32 = 0x0003;

pub const CLUSTER: Cluster<'static> = Cluster::new(
    CLUSTER_ID,
    4,
    0,
    &[rs_matter::dm::Attribute::new(
        AttributeId::IdentifyTime as u32,
        Access::RV,
        Quality::OPTIONAL,
    )],
    &[rs_matter::dm::Command::new(
        CommandId::Identify as u32,
        None,
        Access::NEED_OPERATE,
    )],
    rs_matter::with!(all),
    rs_matter::with!(all),
);

pub trait IdentifyHooks {
    fn on_identify(&self, time: u16);
}

pub struct IdentifyHandler<'a, H: IdentifyHooks> {
    dataver: Dataver,
    hooks: H,
    identify_time: Cell<u16>,
    _phantom: core::marker::PhantomData<&'a ()>,
}

impl<'a, H: IdentifyHooks> IdentifyHandler<'a, H> {
    pub fn new(dataver: Dataver, hooks: H) -> Self {
        Self {
            dataver,
            hooks,
            identify_time: Cell::new(0),
            _phantom: core::marker::PhantomData,
        }
    }

    pub const fn adapt(self) -> identify::HandlerAsyncAdaptor<Self> {
        identify::HandlerAsyncAdaptor(self)
    }
}

impl<H: IdentifyHooks> ClusterAsyncHandler for IdentifyHandler<'_, H> {
    const CLUSTER: Cluster<'static> = CLUSTER;

    fn dataver(&self) -> u32 {
        self.dataver.get()
    }

    fn dataver_changed(&self) {
        self.dataver.changed();
    }

    async fn run(&self, _ctx: impl HandlerContext) -> Result<(), Error> {
        // Simple run loop that doesn't do much for now
        // In a real implementation, we might handle the countdown here
        // For now, we rely on the hook to handle the behavior
        core::future::pending().await
    }

    async fn identify_time(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<u16, rs_matter::error::Error> {
        defmt::info!("identify_time read");
        Ok(self.identify_time.get())
    }

    async fn identify_type(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<IdentifyTypeEnum, rs_matter::error::Error> {
        defmt::info!("identify_type read");
        Ok(IdentifyTypeEnum::LightOutput)
    }

    async fn set_identify_time(
        &self,
        ctx: impl rs_matter::dm::WriteContext,
        value: u16,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("set_identify_time: {}", value);
        self.identify_time.set(value);
        Ok(())
    }

    async fn handle_identify(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: IdentifyRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_identify called");
        if let Ok(time) = request.identify_time() {
            self.hooks.on_identify(time);
        }
        Ok(())
    }

    async fn handle_trigger_effect(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: TriggerEffectRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        defmt::info!("handle_trigger_effect called");
        Ok(())
    }
}
