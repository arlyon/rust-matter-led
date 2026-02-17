use core::cell::Cell;
use rs_matter::{
    dm::{Cluster, Dataver, HandlerContext},
    error::Error,
};

use crate::clusters::decl::identify::{
    self, IdentifyRequest, IdentifyTypeEnum, TriggerEffectRequest,
};

use super::decl::identify::ClusterAsyncHandler;

pub const CLUSTER_ID: u32 = 0x0003;

pub enum AttributeId {
    IdentifyTime = 0x0000,
}

pub enum CommandId {
    Identify = 0x00,
    TriggerEffect = 0x40,
}

pub const CLUSTER: Cluster<'static> = Cluster::new(
    CLUSTER_ID,
    4,
    0,
    &[
        // TODO
    ],
    &[
        // TODO
    ],
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
        todo!()
    }

    async fn identify_type(
        &self,
        ctx: impl rs_matter::dm::ReadContext,
    ) -> Result<IdentifyTypeEnum, rs_matter::error::Error> {
        todo!()
    }

    async fn set_identify_time(
        &self,
        ctx: impl rs_matter::dm::WriteContext,
        value: u16,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_identify(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: IdentifyRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }

    async fn handle_trigger_effect(
        &self,
        ctx: impl rs_matter::dm::InvokeContext,
        request: TriggerEffectRequest<'_>,
    ) -> Result<(), rs_matter::error::Error> {
        todo!()
    }
}
