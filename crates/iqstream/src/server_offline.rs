use crate::config::{Ask, Offered, Public, ServerConfig, StreamConfig, Tune};
use crate::proto::{Setting, SettingValue, StreamDesc};
use common::{Error, Result};
use std::convert::Infallible;
use std::net::SocketAddr;
use std::sync::Arc;

pub struct Stream(Infallible);

impl Stream {
    pub fn id(&self) -> u16 {
        match self.0 {}
    }

    pub fn name(&self) -> &str {
        match self.0 {}
    }

    pub fn sample_rate(&self) -> u32 {
        match self.0 {}
    }

    pub fn set_sample_rate(&self, _: u32) {
        match self.0 {}
    }

    pub fn gain_db(&self) -> Option<f32> {
        match self.0 {}
    }

    pub fn set_gain_db(&self, _: Option<f32>) {
        match self.0 {}
    }

    pub fn settings(&self) -> Vec<Setting> {
        match self.0 {}
    }

    pub fn set_settings(&self, _: Vec<Setting>) {
        match self.0 {}
    }

    pub fn tunable(&self) -> bool {
        match self.0 {}
    }

    pub fn tune_range_hz(&self) -> Option<(u64, u64)> {
        match self.0 {}
    }

    pub fn set_tune_range_hz(&self, _: Option<(u64, u64)>) {
        match self.0 {}
    }

    pub fn subscribers(&self) -> usize {
        match self.0 {}
    }

    pub fn listened(&self) -> bool {
        match self.0 {}
    }

    pub fn blocks_sent(&self) -> u64 {
        match self.0 {}
    }

    pub fn blocks_dropped(&self) -> u64 {
        match self.0 {}
    }

    pub fn center_hz(&self) -> u64 {
        match self.0 {}
    }

    pub fn set_hardware(&self, _: &str) {
        match self.0 {}
    }

    pub fn hardware(&self) -> String {
        match self.0 {}
    }

    pub fn desc(&self) -> StreamDesc {
        match self.0 {}
    }

    pub fn push(&self, _: &[u8]) {
        match self.0 {}
    }

    pub fn ask(&self, _: u64) -> bool {
        match self.0 {}
    }

    pub fn wanted(&self) -> Option<Tune> {
        match self.0 {}
    }

    pub fn ask_setting(&self, _: &str, _: SettingValue) -> bool {
        match self.0 {}
    }

    pub fn asked(&self) -> Vec<Ask> {
        match self.0 {}
    }

    pub fn retuned(&self, _: u64) {
        match self.0 {}
    }
}

pub struct Server(Infallible);

impl Server {
    pub fn start(addr: SocketAddr, _: ServerConfig) -> Result<Arc<Self>> {
        Err(Error::other(format!("{addr}: this build serves no IQStream")))
    }

    pub fn addr(&self) -> SocketAddr {
        match self.0 {}
    }

    pub fn webtransport(&self) -> Option<Offered> {
        match self.0 {}
    }

    pub fn webrtc(&self) -> bool {
        match self.0 {}
    }

    pub fn answer(&self, _: &str) -> Result<String> {
        match self.0 {}
    }

    pub fn set_public(&self, _: Option<Public>) {
        match self.0 {}
    }

    pub fn public(&self) -> Option<Public> {
        match self.0 {}
    }

    pub fn name(&self) -> &str {
        match self.0 {}
    }

    pub fn add_stream(&self, _: StreamConfig) -> Arc<Stream> {
        match self.0 {}
    }

    pub fn stream_named(&self, _: StreamConfig) -> Arc<Stream> {
        match self.0 {}
    }

    pub fn stream(&self, _: u16) -> Option<Arc<Stream>> {
        match self.0 {}
    }

    pub fn default_stream(&self) -> Option<Arc<Stream>> {
        match self.0 {}
    }

    pub fn streams(&self) -> Vec<Arc<Stream>> {
        match self.0 {}
    }

    pub fn remove_stream(&self, _: u16) {
        match self.0 {}
    }
}
